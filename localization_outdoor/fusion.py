"""RTK-aided inertial localization with bounded delayed-measurement replay."""

from collections import deque
import math
import numpy as np
from .eskf import ErrorStateFilter, State, exp_rotation, yaw_jacobian
from .geometry import enu, wrap
from .quality import fix_grade


class FusionFeedback:
    def __init__(self, anchor, config):
        self.anchor, self.config = anchor, config
        self.options = config["fusion"]
        self.filter = None
        self.initialized = False
        self.fix = None
        self.fix_at = -math.inf
        self.last_tow = None
        self.gnss_usable = False
        self.stable_since = None
        self.velocity_valid = False
        self.imu = {}
        self.fault = None
        self.reason = "waiting_for_stationary_imu_and_rtk"
        self.output = None
        self.last_raw_at = None
        self.last_heading_at = -math.inf
        self.window = deque()
        self.pending = []
        self.local_p = np.zeros(3)
        self.local_yaw = 0.0
        self.local_rotation = np.eye(3)
        self.local_attitude = np.eye(3)
        self.recovery_id = 0
        self.recovery_at = None
        self.gap_recoveries = 0
        self.dropped_samples = 0
        self.nav_block = None
        self.last_fixed_at = None
        self.initialized_at = None
        self.inertial_since = None
        self.imu_degraded = False
        self.bridge_heading_at = -math.inf

    @property
    def grade(self):
        if not self.initialized:
            return None
        if (
            self.gnss_usable
            and self.last_raw_at is not None
            and self.output is not None
            and abs(self.output['stamp'] - self.fix_at) < self.config["sensor_timeout_s"]
        ):
            return "fixed" if self.fix["fixed"] else "float"
        return "inertial"

    def invalidate_gnss(self, reason):
        if self.filter is not None and self.gnss_usable:
            self.filter.insert(self.filter.state.t, "gnss_invalid", None)
        self.gnss_usable = False
        self.velocity_valid = False
        self.stable_since = None
        self.reason = reason

    def update_fix(self, fix, received):
        grade = fix_grade(fix, self.config)
        values = [fix.get(k) for k in ("vel_e_mps", "vel_n_mps", "speed_accuracy_mps")]
        if (
            grade is None
            or not all(
                isinstance(v, (int, float))
                and not isinstance(v, bool)
                and math.isfinite(v)
                for v in values
            )
            or not 0 < values[2]
        ):
            self.invalidate_gnss("gnss_quality_unusable")
            return
        tow = fix["tow_ms"]
        if tow == self.last_tow:
            return
        if (
            self.last_tow is not None
            and tow < self.last_tow
            and not (self.last_tow >= 604799000 and tow < 1000)
        ):
            self.invalidate_gnss("gnss_epoch_reversed")
            return
        if self.filter and (
            received < self.filter.base.t
            or received < self.filter.state.t - self.options["history_s"]
        ):
            self.filter.dropped += 1
            return
        lost = (
            not self.gnss_usable
            or received - self.fix_at >= self.config["sensor_timeout_s"]
        )
        if lost:
            self.stable_since = received
        self.last_tow, self.fix_at = tow, received
        self.fix = dict(fix)
        if grade == "fixed":
            self.last_fixed_at = received
        self.gnss_usable, self.velocity_valid = True, True
        if self.initialized:
            data = (
                enu(fix["latitude"], fix["longitude"], self.anchor),
                values[:2],
                fix["hacc_m"],
                values[2],
                lost and grade == "fixed",
            )
            self.pending.append((received, "gnss", data))

    def update_imu(self, observation, now):
        self.imu = dict(observation)
        at = observation.get("angle_at", -math.inf)
        if (
            self.filter
            and observation.get("ready")
            and self.last_heading_at < at
            and now - at < self.config["imu"]["timeout_s"]
        ):
            self.pending.append(
                (
                    at,
                    "heading",
                    (
                        observation["body_yaw_enu_rad"],
                        math.radians(self.config["imu"]["yaw_std_deg"]) ** 2,
                    ),
                )
            )
            self.last_heading_at = at

    def heading_ready(self, now):
        return bool(self.feedback_ready(now))

    def reset_gap(self, reason):
        self.filter = None
        self.initialized = False
        self.output = None
        self.pending.clear()
        self.window.clear()
        self.stable_since = None
        self.gnss_usable = False
        self.fault = reason
        self.reason = "reinitializing_after_imu_gap"
        self.initialized_at = self.last_fixed_at = self.inertial_since = None
        self.imu_degraded = False
        self.bridge_heading_at = -math.inf
        self.gap_recoveries += 1

    def check_timeout(self, now):
        if not self.initialized or now-self.last_raw_at < .15:
            return
        self.imu_degraded = True
        # Missing IMU samples never erase the calibrated estimator. Fresh RTK
        # keeps correcting a constant-velocity prediction without acceleration.
        if not self.gnss_usable or not 0 <= now-self.fix_at < self.config['sensor_timeout_s']:
            return
        old_t = self.filter.state.t
        if now <= old_t:
            return
        self.filter.insert(old_t, 'coast', None)
        for at, kind, data in sorted(self.pending, key=lambda row: row[0]):
            if at <= now and self.filter.insert(at, kind, data) and kind == 'gnss' and data[-1]:
                self.recovery_id += 1
                self.recovery_at = now
        self.pending = [row for row in self.pending if row[0] > now]
        self.filter.insert(now, 'coast', None)
        s = self.filter.state
        # GNSS travel direction supplies degraded heading while moving forward.
        # At rest it cannot observe rotation, so last heading has a grace period.
        speed = math.hypot(self.fix['vel_e_mps'], self.fix['vel_n_mps'])
        if (now-self.last_heading_at >= self.config['imu']['timeout_s']
                and speed >= max(.15, 1.5*self.fix['speed_accuracy_mps'])):
            course = math.atan2(self.fix['vel_n_mps'], self.fix['vel_e_mps'])
            yaw = math.atan2(s.R[1, 0], s.R[0, 0])
            change = max(-.4*(now-old_t), min(.4*(now-old_t), wrap(course-yaw)))
            # Keep this correction in replay history, including its uncertainty.
            self.filter.insert(now, 'heading', (wrap(yaw+change), math.radians(10)**2))
            s = self.filter.state
            corrected_yaw = math.atan2(s.R[1, 0], s.R[0, 0])
            delta = wrap(corrected_yaw-yaw)
            self.local_attitude = exp_rotation(np.array([0., 0., delta])) @ self.local_attitude
            self.local_yaw = wrap(self.local_yaw+delta)
            self.bridge_heading_at = now
        self.local_p += self.local_rotation.T @ s.v * (now-old_t)
        self._output(s.bg)

    def raw(self, acceleration, gyro, t):
        if self.last_raw_at is not None and t <= self.last_raw_at:
            self.dropped_samples += 1
            return
        a, w = np.asarray(acceleration, dtype=float), np.asarray(gyro, dtype=float)
        if not np.isfinite(a).all() or not np.isfinite(w).all():
            self.dropped_samples += 1
            return
        if self.last_raw_at is not None and t - self.last_raw_at >= min(
            0.15, self.config["imu"]["timeout_s"]
        ):
            if self.filter:
                self.filter.insert(self.filter.state.t, 'coast', None)
        previous = self.filter.state.t if self.filter else self.last_raw_at
        self.last_raw_at = t
        if not self.initialized:
            self._initialize(a, w, t)
            if not self.initialized:
                return
        else:
            if not self.filter.insert(t, "inertial", (a, w)):
                self.dropped_samples += 1
                return
            s = self.filter.state
            dt = max(0., t - previous)
            # This local reference only integrates forward; GNSS replay never rewrites it.
            self.local_p += self.local_rotation.T @ s.v * dt
            self.local_attitude = self.local_attitude @ exp_rotation((w - s.bg) * min(dt, .15))
            self.local_yaw = wrap(
                math.atan2(self.local_attitude[1, 0], self.local_attitude[0, 0])
                - math.atan2(self.local_rotation[1, 0], self.local_rotation[0, 0])
            )
        for at, kind, data in sorted(self.pending, key=lambda row: row[0]):
            if at > t:
                continue
            if self.filter.insert(at, kind, data) and kind == "gnss" and data[-1]:
                self.recovery_id += 1
                self.recovery_at = t
        self.pending = [row for row in self.pending if row[0] > t]
        self.imu_degraded = False
        self._output(w)

    def _initialize(self, a, w, t):
        c, o = self.config, self.options
        rotation = self.imu.get("body_rotation")
        angle_at = self.imu.get("angle_at", -math.inf)
        gnss_ok = self.gnss_usable and abs(t - self.fix_at) < c["sensor_timeout_s"]
        heading_ok = (
            self.imu.get("ready")
            and rotation is not None
            and abs(t - angle_at) < c["imu"]["timeout_s"]
        )
        still = (
            np.linalg.norm(w) < o["stationary_gyro_rps"]
            and abs(np.linalg.norm(a) - 9.80665) < o["stationary_accel_mps2"]
        )
        if gnss_ok:
            still = (
                still
                and math.hypot(self.fix["vel_e_mps"], self.fix["vel_n_mps"])
                < o["stationary_speed_mps"]
            )
        if not gnss_ok or not heading_ok or not still:
            self.window.clear()
            self.reason = (
                "heading_quality_unconfirmed"
                if not heading_ok
                else "waiting_for_stationary_imu_and_rtk"
            )
            return
        self.window.append((t, a.copy(), w.copy()))
        while len(self.window) > 1 and self.window[1][0] <= t - o["stationary_s"]:
            self.window.popleft()
        if (
            t - self.window[0][0] < o["stationary_s"] - 1e-6
            or self.stable_since is None
            or t - self.stable_since < c["gnss_stable_s"]
        ):
            return
        samples = np.array([row[1] for row in self.window])
        if max(np.std(samples, axis=0)) > o["stationary_accel_std_mps2"]:
            self.window.clear()
            return
        R = np.asarray(rotation)
        bg = np.mean([row[2] for row in self.window], axis=0)
        ba = samples.mean(axis=0) - R.T @ np.array([0.0, 0.0, 9.80665])
        arm = np.r_[c["antenna_xy_m"], 0.0] - np.asarray(o["imu_xyz_m"])
        p = (
            np.r_[enu(self.fix["latitude"], self.fix["longitude"], self.anchor), 0.0]
            - R @ arm
        )
        v = np.r_[self.fix["vel_e_mps"], self.fix["vel_n_mps"], 0.0] - R @ np.cross(
            w - bg, arm
        )
        variances = (
            [self.fix["hacc_m"] ** 2] * 2
            + [100.0]
            + [self.fix["speed_accuracy_mps"] ** 2] * 2
            + [1.0]
            + [math.radians(2) ** 2] * 2
            + [math.radians(c["imu"]["yaw_std_deg"]) ** 2]
            + [0.005**2] * 3
            + [0.1**2] * 3
        )
        state = State(t, p, v, R, bg, ba, np.diag(variances), a.copy(), w.copy())
        self.filter = ErrorStateFilter(
            state, o, arm, o["history_s"], o["imu_xyz_m"], c["sensor_timeout_s"]
        )
        self.local_p = np.zeros(3)
        self.local_yaw = 0.0
        initial_yaw = math.atan2(R[1, 0], R[0, 0])
        self.local_rotation = exp_rotation(np.array([0.0, 0.0, initial_yaw]))
        self.local_attitude = R.copy()
        self.initialized, self.fault, self.reason = True, None, None
        self.initialized_at = t
        self.window.clear()

    def uncertainty(self):
        if not self.filter:
            return {"horizontal_2sigma_m": None, "yaw_2sigma_deg": None}
        s = self.filter.state
        # Include the IMU-to-base lever arm in horizontal position uncertainty.
        from .eskf import skew

        J = np.zeros((2, 15))
        J[:, :3] = np.eye(3)[:2]
        J[:, 6:9] = (s.R @ skew(self.options["imu_xyz_m"]))[:2]
        covariance = J @ s.P @ J.T
        yaw_gradient = yaw_jacobian(s.R)
        yaw_variance = yaw_gradient @ s.P[6:9, 6:9] @ yaw_gradient
        return {
            "horizontal_2sigma_m": 2
            * math.sqrt(max(0.0, np.linalg.eigvalsh(covariance)[-1])),
            "yaw_2sigma_deg": math.degrees(2 * math.sqrt(max(0.0, yaw_variance))),
        }

    def _output(self, w):
        if self.grade == "inertial":
            if self.inertial_since is None:
                self.inertial_since = self.last_raw_at
        else:
            self.inertial_since = None
        s = self.filter.state
        if not all(np.isfinite(v).all() for v in (s.p, s.v, s.R, s.bg, s.ba, s.P)):
            self.reset_gap("nonfinite_filter_state")
            return
        position = s.p - s.R @ np.asarray(self.options["imu_xyz_m"])
        velocity = s.v - s.R @ np.cross(w - s.bg, self.options["imu_xyz_m"])
        yaw = math.atan2(s.R[1, 0], s.R[0, 0])
        # Planar-only body velocity: the vertical axis has no GNSS
        # observation, so rotating the full 3D velocity into the body frame
        # leaks vertical drift into the planar output whenever the body
        # tilts. Rotate only the ENU horizontal velocity by heading.
        cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
        body_v = np.array(
            [
                cos_yaw * velocity[0] + sin_yaw * velocity[1],
                -sin_yaw * velocity[0] + cos_yaw * velocity[1],
                0.0,
            ]
        )
        # Propagate the same horizontal velocity function into its covariance,
        # including heading, lever arm and gyro-bias uncertainty. Vertical
        # velocity is absent from this function and therefore from its Jacobian.
        from .eskf import skew

        yaw_rot = np.array([[cos_yaw, sin_yaw], [-sin_yaw, cos_yaw]])
        arm = np.asarray(self.options["imu_xyz_m"])
        arm_velocity = np.cross(w - s.bg, arm)
        J = np.zeros((3, 15))
        J[:, 3:6] = np.eye(3)
        J[:, 6:9] = s.R @ skew(arm_velocity)
        J[:, 9:12] = -s.R @ skew(arm)
        planar_J = yaw_rot @ J[:2]
        planar_J[:, 6:9] += np.outer([body_v[1], -body_v[0]], yaw_jacobian(s.R))
        planar_cov = planar_J @ s.P @ planar_J.T
        gyro_J = -yaw_rot @ (s.R @ skew(arm))[:2]
        planar_cov += gyro_J @ gyro_J.T * self.options["gyro_noise"] ** 2
        planar_cov = (planar_cov + planar_cov.T) / 2
        planar_std = math.sqrt(max(0.0, np.linalg.eigvalsh(planar_cov)[-1]))
        self.output = {
            "stamp": s.t,
            "odom_pose": {
                "x": float(self.local_p[0]),
                "y": float(self.local_p[1]),
                "yaw": self.local_yaw,
            },
            "map_pose": {
                "x": float(position[0] + self.config["map_offset_xy_m"][0]),
                "y": float(position[1] + self.config["map_offset_xy_m"][1]),
                "yaw": yaw,
            },
            "twist": {
                "linear_x": float(body_v[0]),
                "linear_y": float(body_v[1]),
                "angular_z": float((w - s.bg)[2]),
            },
            "prediction_s": max(0.0, s.t - self.fix_at),
            "velocity_accuracy_mps": float(planar_std),
            "velocity_covariance": planar_cov.tolist(),
        }
        self.reason = None

    def feedback_ready(self, now):
        heading_at = max(self.last_heading_at, self.bridge_heading_at)
        degraded_ok = (not self.imu_degraded or (
            0 <= now-self.last_raw_at < self.config['imu']['timeout_s']
        ) or (
            self.gnss_usable and 0 <= now-self.fix_at < self.config['sensor_timeout_s']
            and 0 <= now-heading_at < self.config['imu']['timeout_s']))
        return bool(
            self.initialized
            and self.output
            and not self.fault
            and degraded_ok
            and 0 <= now - self.output["stamp"] < self.config["imu"]["timeout_s"]
        )

    def ready(self, now):
        uncertainty = self.uncertainty()
        self.nav_block = None
        if not self.feedback_ready(now):
            self.nav_block = self.fault or "initializing"
        elif (
            uncertainty["horizontal_2sigma_m"] > self.options["max_horizontal_2sigma_m"]
            or uncertainty["yaw_2sigma_deg"] > self.options["max_yaw_2sigma_deg"]
        ):
            self.nav_block = "inertial_uncertainty_exceeded"
        elif not all(
            self.config[k]
            for k in ("calibrated", "antenna_offset_confirmed", "feedback_confirmed")
        ):
            self.nav_block = "calibration_unconfirmed"
        return self.nav_block is None

    def sample(self, now):
        return self.output if self.feedback_ready(now) else None

    def diagnostics(self, now):
        ready = self.ready(now)
        return {
            "stage": ("fault" if self.fault else self.grade or "initializing"),
            "rtk_grade": ("fixed" if self.fix["fixed"] else "float")
            if self.gnss_usable
            and self.fix
            and 0 <= now - self.fix_at < self.config["sensor_timeout_s"]
            else "unavailable",
            "estimate_valid": self.feedback_ready(now),
            "stationary_collected_s": self.window[-1][0] - self.window[0][0]
            if self.window
            else 0.0,
            "navigation_ready": ready,
            "navigation_block": self.nav_block,
            "outage_s": max(0.0, now - self.fix_at) if self.fix else None,
            "uncertainty": self.uncertainty(),
            "recovery_id": self.recovery_id,
            "recovery_at": self.recovery_at,
            "replayed_updates": self.filter.replays if self.filter else 0,
            "dropped_samples": self.dropped_samples,
            "gap_recoveries": self.gap_recoveries,
            "inertial": self.grade == "inertial",
            "inertial_elapsed_s": max(0.0, now - self.inertial_since)
            if self.inertial_since is not None and self.initialized
            else 0.0,
            "fusion_elapsed_s": max(
                0.0,
                now
                - (
                    self.last_fixed_at
                    if self.last_fixed_at is not None
                    else self.initialized_at
                ),
            )
            if self.initialized_at is not None
            else None,
            "fusion_timer_reference": "last_fixed"
            if self.last_fixed_at is not None
            else "initialization",
            "covariance_source": "eskf",
            "velocity_source": "rtk_aided_coast" if self.imu_degraded else "inertial_estimate",
            "imu_degraded": self.imu_degraded,
            "heading_source": ("gnss_course" if self.bridge_heading_at > self.last_heading_at
                               else "held_imu" if self.imu_degraded else "imu"),
        }
