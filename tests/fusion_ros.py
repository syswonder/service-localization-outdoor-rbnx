"""Two-process synthetic ROS fusion acceptance; no hardware or motion topics."""

import argparse
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from urllib.request import Request, urlopen


def main():
    import rclpy
    from geometry_msgs.msg import TwistStamped, Vector3Stamped
    from sensor_msgs.msg import Imu, NavSatFix
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", required=True)
    args = p.parse_args()
    c = {
        "origin": {"latitude": 30.0, "longitude": 120.0},
        "estimator_mode": "eskf",
        "calibrated": True,
        "antenna_offset_confirmed": True,
        "feedback_confirmed": True,
        "imu": {
            "base_from_imu_rotation": [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            "mounting_confirmed": True,
            "reference_confirmed": True,
            "magnetic_declination_rad": 0.0,
            "quality_confirmed": True,
            "yaw_std_deg": 2.0,
        },
        "http_port": 19989,
        "topic_remap": {
            k: "/fusion_test/" + k
            for k in (
                "gnss_fix",
                "gnss_velocity",
                "rtk_status",
                "imu",
                "gyro",
                "imu_status",
                "inertial",
            )
        },
        "output_topics": {
            "odom": "/fusion_test/odom",
            "pose": "/fusion_test/pose",
            "status": "/fusion_test/status",
            "tf": "/fusion_test/tf",
            "tf_static": "/fusion_test/tf_static",
        },
    }
    stages = set()
    readiness = {}
    max_inertial = 0.0
    recovered = False
    odom = []
    last_timer = None
    last_status = None
    initializing_rtk_visible = False
    with tempfile.TemporaryDirectory(prefix="fusion-ros-") as directory:
        cfg = Path(directory) / "config.json"
        cfg.write_text(json.dumps(c))
        log = (Path(directory) / "worker.log").open("w+")
        child = subprocess.Popen(
            [sys.executable, "-m", "localization_outdoor.node", "--config", str(cfg)],
            stdout=log,
            stderr=log,
        )
        rclpy.init()
        node = rclpy.create_node("synthetic_fusion_acceptance")
        types = {
            "gnss_fix": NavSatFix,
            "gnss_velocity": TwistStamped,
            "rtk_status": String,
            "imu": Imu,
            "gyro": Vector3Stamped,
            "imu_status": String,
            "inertial": Imu,
        }
        pubs = {
            k: node.create_publisher(v, c["topic_remap"][k], 500)
            for k, v in types.items()
        }

        def status(msg):
            nonlocal \
                max_inertial, \
                recovered, \
                last_timer, \
                last_status, \
                initializing_rtk_visible
            value = json.loads(msg.data)
            last_status = value
            fusion = value.get("fusion") or {}
            stage = fusion.get("stage")
            stages.add(stage)
            if stage == "initializing" and fusion.get("rtk_grade") == "fixed":
                initializing_rtk_visible = True
            readiness[stage] = readiness.get(stage, False) or value["ready"]
            max_inertial = max(max_inertial, fusion.get("inertial_elapsed_s", 0.0))
            if fusion.get("recovery_id", 0) > 0:
                recovered = True
                last_timer = fusion.get("fusion_elapsed_s")

        node.create_subscription(String, c["output_topics"]["status"], status, 500)
        node.create_subscription(
            Odometry,
            c["output_topics"]["odom"],
            lambda m: odom.append(m.pose.pose.position.x),
            500,
        )
        try:
            deadline = time.monotonic() + 5
            while True:
                if child.poll() is not None:
                    raise RuntimeError("Localization worker exited")
                try:
                    with urlopen(
                        Request(
                            "http://127.0.0.1:19989/active",
                            data=b'{"active":true}',
                            headers={"Content-Type": "application/json"},
                        ),
                        timeout=0.2,
                    ):
                        break
                except OSError:
                    if time.monotonic() > deadline:
                        raise
                    rclpy.spin_once(node, timeout_sec=0.02)
            start = time.monotonic()
            last_gnss = -math.inf
            sequence = 0
            while time.monotonic() - start < 9.5:
                elapsed = time.monotonic() - start
                stamp = node.get_clock().now().to_msg()
                ros = stamp.sec + stamp.nanosec / 1e9

                def header(msg, frame="imu"):
                    msg.header.stamp = stamp
                    msg.header.frame_id = frame
                    return msg

                imu = header(Imu())
                imu.orientation.w = 1.0
                imu.linear_acceleration.z = 9.80665
                raw = header(Imu())
                raw.orientation_covariance[0] = -1.0
                raw.linear_acceleration.z = 9.80665
                pubs["imu"].publish(imu)
                pubs["inertial"].publish(raw)
                pubs["gyro"].publish(header(Vector3Stamped()))
                if elapsed - last_gnss >= 0.1:
                    last_gnss = elapsed
                    sequence += 1
                    quality = (
                        4
                        if elapsed < 6.0 or elapsed >= 8.0
                        else 5
                        if elapsed < 6.6
                        else 0
                    )
                    fix = header(NavSatFix(), "gnss")
                    fix.status.status = 2 if quality else -1
                    fix.latitude = 30.0
                    fix.longitude = 120.0 + (0.001 if elapsed >= 8.0 else 0.0)
                    fix.position_covariance_type = 2
                    fix.position_covariance = [
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.01,
                        0.0,
                        0.0,
                        0.0,
                        0.04,
                    ]
                    velocity = header(TwistStamped(), "gnss_enu")
                    measurement = round(ros * 1000) % 604800000
                    telemetry = {
                        "receiver_quality": quality,
                        "state": {4: "RTK_FIXED", 5: "RTK_FLOAT", 0: "NO_FIX"}[quality],
                        "fix_age_s": 0.0,
                        "measurement_age_s": 0.0,
                        "fix_stamp_s": ros,
                        "epoch_source": "UBX_iTOW_ms",
                        "measurement_id": measurement,
                        "velocity": {
                            "fresh": True,
                            "age_s": 0.0,
                            "stamp_s": ros,
                            "measurement_id": measurement,
                            "accuracy_source": "UBX_NAV_PVT",
                            "accuracy_mps": 0.02,
                        },
                    }
                    pubs["gnss_fix"].publish(fix)
                    pubs["gnss_velocity"].publish(velocity)
                    pubs["rtk_status"].publish(String(data=json.dumps(telemetry)))
                    pubs["imu_status"].publish(
                        String(
                            data=json.dumps(
                                {
                                    "serial_connected": True,
                                    "valid": True,
                                    "generation": "synthetic",
                                }
                            )
                        )
                    )
                for _ in range(4):
                    rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.008)
            result = {
                "synthetic": True,
                "stages": sorted(str(v) for v in stages),
                "ready_by_stage": readiness,
                "max_inertial_elapsed_s": max_inertial,
                "fixed_recovered": recovered,
                "timer_after_recovery_s": last_timer,
                "odom_samples": len(odom),
                "largest_odom_step_m": max(
                    (abs(b - a) for a, b in zip(odom, odom[1:])), default=None
                ),
                "motion_publishers_created": 0,
                "initializing_rtk_visible": initializing_rtk_visible,
                "last_status": last_status,
            }
            result["passed"] = (
                all(s in stages for s in ("initializing", "fixed", "float", "inertial"))
                and all(readiness.get(s) for s in ("fixed", "float", "inertial"))
                and recovered
                and max_inertial > 0.5
                and last_timer is not None
                and last_timer < 0.5
                and result["largest_odom_step_m"] < 1.0
                and len(odom) >= 120
                and initializing_rtk_visible
                and last_status["fusion"]["stage"] == "fixed"
                and last_status["diagnostic"]["input_ages"]["inertial"] < 0.2
            )
            Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
            print(json.dumps(result, indent=2))
            if not result["passed"]:
                raise RuntimeError("Synthetic fusion acceptance failed")
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read(), file=sys.stderr)
            raise
        finally:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
            node.destroy_node()
            rclpy.shutdown()
            log.close()


if __name__ == "__main__":
    main()
