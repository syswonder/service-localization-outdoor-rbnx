"""ROS-shaped observations -> estimator input. Never reads device bytes."""

import math
import numpy as np
from .geometry import quaternion_matrix, wrap
from .estimator import RtkImuFeedback


def seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


class Inputs:
    def __init__(self, config):
        self.config = config
        self.engine = RtkImuFeedback(config["origin"], config)
        self.values = {}
        self.history = {"gnss_fix": {}, "gnss_velocity": {}}
        self.last_epoch = None
        self.day = 0
        self.reason = "waiting_for_primitive_data"

    def put(self, role, message, now, ros_now):
        stamp = seconds(message.header.stamp) if hasattr(message, "header") else ros_now
        age = ros_now - stamp
        if not math.isfinite(age) or age < -0.1:
            return
        self.values[role] = (message, now - max(0.0, age))
        if role in self.history:
            self.history[role][stamp] = self.values[role]
            while len(self.history[role]) > 40:
                self.history[role].pop(next(iter(self.history[role])))

    def fresh(self, role, now, limit=None):
        item = self.values.get(role)
        return (
            item[0]
            if item and 0 <= now - item[1] < (limit or self.config["sensor_timeout_s"])
            else None
        )

    def invalidate(self, reason):
        self.reason = reason
        self.engine.grade = None
        self.engine.velocity_valid = False
        self.engine.stable_since = None

    def step(self, now):
        c = self.config
        t = c["sensor_timeout_s"]
        e = self.engine
        fix = self.fresh("gnss_fix", now)
        rtk = self.fresh("rtk_status", now)
        velocity = self.fresh("gnss_velocity", now)
        fix_at = self.values.get("gnss_fix", (None, -math.inf))[1]
        if rtk is not None:
            fp = self.history["gnss_fix"].get(rtk.get("fix_stamp_s"))
            vp = self.history["gnss_velocity"].get(
                rtk.get("velocity", {}).get("stamp_s")
            )
            fix, fix_at = fp if fp and 0 <= now - fp[1] < t else (None, -math.inf)
            velocity = vp[0] if vp and 0 <= now - vp[1] < t else None
        if fix is None or rtk is None or velocity is None:
            self.invalidate("gnss_stream_stale")
        else:
            quality = rtk.get("receiver_quality")
            v = rtk.get("velocity", {})
            fresh = all(
                isinstance(rtk.get(k), (int, float))
                and not isinstance(rtk[k], bool)
                and 0 <= rtk[k] + now - self.values["rtk_status"][1] < t
                for k in ("fix_age_s", "measurement_age_s")
            )
            vf = v.get("age_s")
            vfresh = (
                v.get("fresh") is True
                and isinstance(vf, (int, float))
                and not isinstance(vf, bool)
                and 0 <= vf + now - self.values["rtk_status"][1] < t
            )
            if (
                not fresh
                or not vfresh
                or rtk.get("state") not in ("RTK_FIXED", "RTK_FLOAT")
                or quality not in (4, 5)
                or fix.status.status < 0
            ):
                self.invalidate("gnss_quality_or_epoch_stale")
            elif fix.position_covariance_type == 0:
                self.invalidate(
                    "position_uncertainty_unknown; require receiver GST or equivalent"
                )
            elif not c["velocity_accuracy_confirmed"]:
                self.invalidate(
                    "velocity_accuracy_unverified; receiver RMC does not report uncertainty"
                )
            elif velocity.header.frame_id != "gnss_enu":
                self.invalidate("velocity_frame_not_ENU")
            else:
                try:

                    def utc(value):
                        if not isinstance(value, str) or len(value) < 6:
                            raise ValueError("Missing measurement UTC")
                        h, m, s = int(value[:2]), int(value[2:4]), float(value[4:])
                        if not 0 <= h < 24 or not 0 <= m < 60 or not 0 <= s < 61:
                            raise ValueError("Invalid UTC")
                        return h * 3600 + m * 60 + s

                    epoch = utc(rtk.get("utc_time"))
                    if abs(epoch - utc(v.get("utc_time"))) > 0.001:
                        raise ValueError("Position/velocity epochs differ")
                    if self.last_epoch is not None and epoch < self.last_epoch:
                        if self.last_epoch > 86399 and epoch < 1:
                            self.day += 1
                        else:
                            raise ValueError("Receiver epoch reversed")
                    self.last_epoch = epoch
                    cov = fix.position_covariance
                    if not all(math.isfinite(cov[i]) and cov[i] > 0 for i in (0, 4)):
                        raise ValueError("Invalid covariance")
                    p = {
                        "latitude": fix.latitude,
                        "longitude": fix.longitude,
                        "hacc_m": math.sqrt(max(cov[0], cov[4])),
                        "valid": True,
                        "differential": True,
                        "fixed": quality == 4,
                        "float": quality == 5,
                        "gga_quality": quality,
                        "correction_age_s": rtk.get("differential_age_s"),
                        "vel_e_mps": velocity.twist.linear.x,
                        "vel_n_mps": velocity.twist.linear.y,
                        "speed_accuracy_mps": c["velocity_accuracy_mps"],
                        "tow_ms": round((self.day * 86400 + epoch) * 1000),
                    }
                    e.update_fix(p, fix_at)
                    self.reason = e.reason
                except (ValueError, TypeError, OverflowError):
                    self.invalidate("invalid_gnss_measurement")
        imu = self.fresh("imu", now, c["imu"]["timeout_s"])
        gyro = self.fresh("gyro", now, c["imu"]["timeout_s"])
        status = self.fresh("imu_status", now, max(1.5, t))
        mounted = (
            c["imu"]["mounting_confirmed"]
            and c["imu"]["base_from_imu_rotation"] is not None
        )
        body_yaw = body_rate = None
        angle_at = self.values.get("imu", (None, -math.inf))[1]
        gyro_at = self.values.get("gyro", (None, -math.inf))[1]
        if mounted:
            rotation = np.asarray(c["imu"]["base_from_imu_rotation"])
            if gyro is not None and status and status.get("serial_connected") is True:
                raw = np.array([gyro.vector.x, gyro.vector.y, gyro.vector.z])
                if np.isfinite(raw).all():
                    body_rate = float((rotation @ raw)[2])
            if (
                imu is not None
                and status
                and status.get("valid") is True
                and imu.orientation_covariance[0] != -1
            ):
                q = imu.orientation
                norm = sum(x * x for x in (q.x, q.y, q.z, q.w))
                if (
                    math.isfinite(norm)
                    and abs(norm - 1) < 0.05
                    and c["imu"]["reference_confirmed"]
                ):
                    world_sensor = quaternion_matrix(
                        q.x / math.sqrt(norm),
                        q.y / math.sqrt(norm),
                        q.z / math.sqrt(norm),
                        q.w / math.sqrt(norm),
                    )
                    world_body = world_sensor @ rotation.T
                    body_yaw = wrap(
                        math.atan2(world_body[1, 0], world_body[0, 0])
                        - c["imu"]["magnetic_declination_rad"]
                    )
        ready = body_yaw is not None and c["imu"]["quality_confirmed"]
        observation = {
            "ready": ready,
            "body_yaw_enu_rad": body_yaw,
            "body_yaw_rate_rps": body_rate,
            "angle_at": angle_at,
            "gyro_at": gyro_at,
        }
        e.update_imu(observation, now)
        return observation
