"""Localization worker consuming only ROS primitive contracts; no device I/O."""

import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import (
    TransformStamped,
    PoseWithCovarianceStamped,
    TwistStamped,
    Vector3Stamped,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from .configuration import validate
from .inputs import Inputs
from .geometry import wrap


def tf_key(t):
    p, q = t.transform.translation, t.transform.rotation
    return (
        t.header.frame_id,
        t.child_frame_id,
        t.header.stamp.sec,
        t.header.stamp.nanosec,
        p.x,
        p.y,
        p.z,
        q.x,
        q.y,
        q.z,
        q.w,
    )


class Localizer(Node):
    def __init__(self, config):
        super().__init__("outdoor_localization")
        self.config = config
        self.inputs = Inputs(config)
        self.active = False
        self.last_publish = -math.inf
        self.sent = deque(maxlen=300)
        self.conflict = False
        out = config["output_topics"]
        self.odom = self.create_publisher(Odometry, out["odom"], 10)
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, out["pose"], 10
        )
        self.status_pub = self.create_publisher(String, out["status"], 10)
        self.tf_pub = self.create_publisher(TFMessage, out["tf"], 50)
        self.create_subscription(TFMessage, out["tf"], self.on_tf, 100)
        types = {
            "gnss_fix": NavSatFix,
            "gnss_velocity": TwistStamped,
            "rtk_status": String,
            "imu": Imu,
            "gyro": Vector3Stamped,
            "imu_status": String,
        }
        for role, typ in types.items():
            self.create_subscription(
                typ,
                config["topic_remap"][role],
                lambda m, r=role: self.receive(r, m),
                qos_profile_sensor_data,
            )
        self.create_timer(0.02, self.tick)
        from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path != "/status":
                    self.send_error(404)
                    return
                data = json.dumps(owner.snapshot(), allow_nan=False).encode()
                self.send_response(200)
                self.end_headers()
                self.wfile.write(data)

            def do_POST(self):
                if self.path != "/active":
                    self.send_error(404)
                    return
                payload = json.loads(
                    self.rfile.read(int(self.headers.get("Content-Length", "0")))
                )
                if type(payload.get("active")) is not bool:
                    self.send_error(400)
                    return
                owner.active = payload["active"]
                self.send_response(200)
                self.end_headers()

            def log_message(self, *_):
                pass

        self.http = ThreadingHTTPServer(
            ("127.0.0.1", config.get("http_port", 19980)), Handler
        )
        threading.Thread(target=self.http.serve_forever, daemon=True).start()

    def receive(self, role, msg):
        if role in ("rtk_status", "imu_status"):
            try:
                msg = json.loads(msg.data)
                if not isinstance(msg, dict):
                    raise ValueError("Expected object")
            except (ValueError, TypeError):
                self.inputs.values.pop(role, None)
                return
        self.inputs.put(
            role, msg, time.monotonic(), self.get_clock().now().nanoseconds / 1e9
        )

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.child_frame_id in ("base_link", "odom") and tf_key(t) not in self.sent:
                self.conflict = True

    def transform(self, stamp, parent, child, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.z = math.sin(yaw / 2)
        t.transform.rotation.w = math.cos(yaw / 2)
        self.sent.append(tf_key(t))
        return t

    def snapshot(self):
        now = time.monotonic()
        e = self.inputs.engine
        c = self.config
        bound = not c["require_map_reference"] or bool(c.get("map_id"))
        ready = bound and e.ready(now) and self.active and not self.conflict
        sample = e.sample(now) if bound else None
        return {
            "ready": bool(ready),
            "coordinate_reference": {
                "frame_id": "map",
                "axes": "east_north",
                "origin_wgs84": dict(c["origin"]) if bound else None,
                "map_id": c.get("map_id"),
                "version": c.get("map_version"),
                "reference_point": "base_link",
                "map_offset_xy_m": list(c["map_offset_xy_m"]),
                "antenna_xy_m": list(c["antenna_xy_m"]),
            },
            "grade": e.grade,
            "fault": ("Load an outdoor map to bind localization" if not bound else None) or e.fault
            or (None if ready else self.inputs.reason or "localization_unready"),
            "ownership_conflict": self.conflict,
            "active": self.active,
            "diagnostic": {
                "input_reason": self.inputs.reason,
                "estimator_reason": e.reason,
                "fix_at": e.fix_at if math.isfinite(e.fix_at) else None,
                "stable_since": e.stable_since,
                "velocity_valid": e.velocity_valid,
                "initialized": e.initialized,
                "input_ages": {
                    role: now - at for role, (_, at) in self.inputs.values.items()
                },
            },
            "heading": {
                "ready": e.heading_ready(now),
                "source": "imu_primitive",
                "body_yaw_enu_rad": e.imu.get("body_yaw_enu_rad"),
                "angle_age_s": None
                if "imu" not in self.inputs.values
                else now - self.inputs.values["imu"][1],
            },
            "feedback": {
                "ready": e.feedback_ready(now),
                "confirmed": c["feedback_confirmed"],
                "source": str(self.inputs.values.get("rtk_status", ({},0))[0].get("velocity",{}).get("source", "unknown"))+"+imu_gyro",
                "velocity_accuracy_source": "receiver_UBX_NAV_PVT" if self.inputs.values.get("rtk_status", ({},0))[0].get("velocity",{}).get("accuracy_source")=="UBX_NAV_PVT" else "deployment_verified_bound",
                "speed_accuracy_mps": e.fix.get("speed_accuracy_mps") if e.fix else None,
            },
            "position_source": "gnss_primitive",
            "position_uncertainty_source": self.inputs.values.get("rtk_status", ({},0))[0].get("position_covariance_source", "UNKNOWN"),
            "velocity_source": self.inputs.values.get("rtk_status", ({},0))[0].get("velocity",{}).get("source", "UNKNOWN"),
            "extrinsics": {
                "ready": c["antenna_offset_confirmed"]
                and c["imu"]["mounting_confirmed"]
            },
            "pose": sample["map_pose"] if sample and e.heading_ready(now) else None,
        }

    def tick(self):
        now = time.monotonic()
        if self.config["require_map_reference"] and not self.config.get("map_id"):
            self.status_pub.publish(String(data=json.dumps(self.snapshot(), allow_nan=False)))
            return
        self.inputs.step(now)
        e = self.inputs.engine
        if self.count_publishers(self.odom.topic_name) > 1:
            self.conflict = True
        sample = e.sample(now)
        if sample and sample["stamp"] > self.last_publish:
            self.last_publish = sample["stamp"]
            from rclpy.duration import Duration

            stamp = (
                self.get_clock().now()
                - Duration(seconds=max(0.0, now - sample["stamp"]))
            ).to_msg()
            p, v = sample["odom_pose"], sample["twist"]
            o = Odometry()
            o.header.stamp = stamp
            o.header.frame_id = "odom"
            o.child_frame_id = "base_link"
            o.pose.pose.position.x = p["x"]
            o.pose.pose.position.y = p["y"]
            o.pose.pose.orientation.z = math.sin(p["yaw"] / 2)
            o.pose.pose.orientation.w = math.cos(p["yaw"] / 2)
            o.twist.twist.linear.x = v["linear_x"]
            o.twist.twist.linear.y = v["linear_y"]
            o.twist.twist.angular.z = v["angular_z"]
            o.pose.covariance[0] = o.pose.covariance[7] = e.fix["hacc_m"] ** 2
            o.pose.covariance[35] = (
                math.radians(self.config["imu"]["yaw_std_deg"] or 180) ** 2
            )
            o.twist.covariance[0] = o.twist.covariance[7] = (
                e.fix["speed_accuracy_mps"] ** 2
            )
            o.twist.covariance[35] = 1e6
            for i in (14, 21, 28):
                o.pose.covariance[i] = o.twist.covariance[i] = 1e6
            self.odom.publish(o)
            transforms = [
                self.transform(stamp, "odom", "base_link", p["x"], p["y"], p["yaw"])
            ]
            if e.ready(now) and not self.conflict:
                g = sample["map_pose"]
                angle = wrap(g["yaw"] - p["yaw"])
                c, s = math.cos(angle), math.sin(angle)
                transforms.append(
                    self.transform(
                        stamp,
                        "map",
                        "odom",
                        g["x"] - c * p["x"] + s * p["y"],
                        g["y"] - s * p["x"] - c * p["y"],
                        angle,
                    )
                )
                msg = PoseWithCovarianceStamped()
                msg.header.stamp = stamp
                msg.header.frame_id = "map"
                msg.pose.pose.position.x = g["x"]
                msg.pose.pose.position.y = g["y"]
                msg.pose.pose.orientation.z = math.sin(g["yaw"] / 2)
                msg.pose.pose.orientation.w = math.cos(g["yaw"] / 2)
                msg.pose.covariance = o.pose.covariance
                self.pose_pub.publish(msg)
            if not self.conflict:
                self.tf_pub.publish(TFMessage(transforms=transforms))
        self.status_pub.publish(
            String(data=json.dumps(self.snapshot(), allow_nan=False))
        )

    def destroy_node(self):
        self.active = False
        self.http.shutdown()
        self.http.server_close()
        super().destroy_node()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--config", required=True)
    args = p.parse_args()
    c = validate(json.loads(Path(args.config).read_text()))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Localizer(c)
    stopping = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while not stopping:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
