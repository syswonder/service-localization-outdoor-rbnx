"""No device paths, NTRIP, vendor decoder or business map dependency."""

import math
import re
import numpy as np

FUSION_DEFAULTS = {
    "stationary_s": 5.0, "history_s": 3.0,
    "stationary_gyro_rps": 0.03, "stationary_accel_mps2": 0.5,
    "stationary_accel_std_mps2": 0.1, "stationary_speed_mps": 0.05,
    "zupt_velocity_std_mps": 0.05,
    "accel_noise": 0.15, "gyro_noise": 0.01,
    "gyro_bias_walk": 0.0005, "accel_bias_walk": 0.005,
    "max_horizontal_2sigma_m": 3.0, "max_yaw_2sigma_deg": 30.0,
    "imu_xyz_m": [0.0, 0.0, 0.0],
}

DEFAULTS = {
    "estimator_mode": "legacy",
    "fusion": FUSION_DEFAULTS,
    "sensor_timeout_s": 3.0,
    "gnss_stable_s": 2.0,
    "gnss_fixed_hacc_m": 3.0,
    "gnss_float_hacc_m": 3.0,
    # Deprecated compatibility key; reported uncertainty never rejects a fix.
    "gnss_speed_accuracy_mps": None,
    "require_correction_age": False,
    "correction_max_age_s": 5.0,
    "calibrated": False,
    "antenna_offset_confirmed": False,
    "feedback_confirmed": False,
    "require_map_reference": False,
    "velocity_accuracy_mps": None,
    "velocity_accuracy_confirmed": False,
    "antenna_xy_m": [0.0, 0.0],
    "map_offset_xy_m": [0.0, 0.0],
    "imu": {
        "base_from_imu_rotation": None,
        "mounting_confirmed": False,
        "reference_confirmed": False,
        "magnetic_declination_rad": None,
        # Default yaw uncertainty from the 2026-11-11 field authorisation
        # (observed magnetic-heading error ≈ 18°, 20° assumed); no longer a
        # per-boot confirmation gate, only an observation variance.
        "yaw_std_deg": 20.0,
        "timeout_s": 3.0,
        "course_heading": False,
    },
    "output_topics": {
        "odom": "/localization/odom",
        "pose": "/localization/pose",
        "status": "/localization/status",
        "tf": "/tf",
        "tf_static": "/tf_static",
    },
}
CONTRACTS = {
    "gnss_fix": ("gnss", "robonix/primitive/gnss/fix"),
    "rtk_status": ("gnss", "robonix/primitive/gnss/rtk_status"),
    "gnss_velocity": ("gnss", "robonix/primitive/gnss/velocity"),
    "imu": ("imu", "robonix/primitive/imu/imu"),
    "gyro": ("imu", "robonix/primitive/imu/angular_velocity"),
    "imu_status": ("imu", "robonix/primitive/imu/status"),
    "inertial": ("imu", "robonix/primitive/imu/inertial"),
}


def finite(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def validate(raw):
    if not isinstance(raw, dict):
        raise ValueError("Configuration must be an object")
    allowed = set(DEFAULTS) | {
        "origin",
        "provider_ids",
        "topic_remap",
        "run_dir",
        "http_port",
        "worker_python",
        "map_id",
        "map_version",
    }
    if set(raw) - allowed:
        raise ValueError("Unknown configuration fields")
    c = {
        **DEFAULTS,
        **raw,
        "imu": {**DEFAULTS["imu"], **raw.get("imu", {})},
        "fusion": {**FUSION_DEFAULTS, **raw.get("fusion", {})},
        "output_topics": {**DEFAULTS["output_topics"], **raw.get("output_topics", {})},
    }
    if raw.get("map_id") or raw.get("map_version"):
        if (not isinstance(raw.get("map_id"), str)
                or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", raw["map_id"])
                or not isinstance(raw.get("map_version"), str)
                or not re.fullmatch(r"[a-f0-9]{16,64}", raw["map_version"])):
            raise ValueError("Map ID and content version must be supplied together")
    o = c.get("origin")
    if (
        not isinstance(o, dict)
        or set(o) != {"latitude", "longitude"}
        or not all(finite(v) for v in o.values())
        or not -90 < o["latitude"] < 90
        or not -180 <= o["longitude"] <= 180
    ):
        raise ValueError("An explicit WGS84 map origin is required")
    for key, d in DEFAULTS.items():
        if isinstance(d, bool) and type(c[key]) is not bool:
            raise ValueError(key + " must be boolean")
        if isinstance(d, float) and (not finite(c[key]) or c[key] <= 0):
            raise ValueError(key + " must be positive")
    for key in ("antenna_xy_m", "map_offset_xy_m"):
        if (
            not isinstance(c[key], list)
            or len(c[key]) != 2
            or not all(finite(v) for v in c[key])
        ):
            raise ValueError("Invalid " + key)
    imu = c["imu"]
    if set(imu) - DEFAULTS["imu"].keys():
        raise ValueError("Unknown IMU configuration")
    for key in (
        "mounting_confirmed",
        "reference_confirmed",
        "course_heading",
    ):
        if type(imu[key]) is not bool:
            raise ValueError("IMU confirmation must be boolean")
    for key in ("magnetic_declination_rad", "yaw_std_deg"):
        if imu[key] is not None and not finite(imu[key]):
            raise ValueError("Invalid " + key)
    if imu["yaw_std_deg"] is None:
        imu["yaw_std_deg"] = 20.0
    if not finite(imu["timeout_s"]) or not 0 < imu["timeout_s"] <= 3:
        raise ValueError("Invalid IMU timeout")
    rotation = imu["base_from_imu_rotation"]
    if rotation is not None:
        r = np.asarray(rotation, dtype=float)
        if (
            r.shape != (3, 3)
            or not np.isfinite(r).all()
            or not np.allclose(r @ r.T, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1)
        ):
            raise ValueError("Invalid IMU mounting rotation")
    if imu["mounting_confirmed"] and rotation is None:
        raise ValueError("Confirmed mounting requires rotation")
    if imu["reference_confirmed"] and imu["magnetic_declination_rad"] is None:
        raise ValueError("North reference correction required")
    if c["velocity_accuracy_mps"] is not None and (
        not finite(c["velocity_accuracy_mps"]) or c["velocity_accuracy_mps"] <= 0
    ):
        raise ValueError("Invalid velocity uncertainty")
    if c["velocity_accuracy_confirmed"] and c["velocity_accuracy_mps"] is None:
        raise ValueError("A verified velocity bound is required")
    for t in [*c["output_topics"].values(), *c.get("topic_remap", {}).values()]:
        if not isinstance(t, str) or not re.fullmatch(
            r"/[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*", t
        ):
            raise ValueError("Invalid topic")
    if set(c.get("topic_remap", {})) - CONTRACTS.keys():
        raise ValueError("Unknown input role")
    if len(set(c["output_topics"].values())) != len(c["output_topics"]):
        raise ValueError("Output topics must be distinct")
    if c["estimator_mode"] not in ("legacy", "eskf"):
        raise ValueError("Unsupported estimator_mode")
    f = c["fusion"]
    if set(f)-FUSION_DEFAULTS.keys():
        raise ValueError("Unknown fusion configuration")
    for key in FUSION_DEFAULTS:
        if key != "imu_xyz_m" and (not finite(f[key]) or f[key] <= 0):
            raise ValueError("Invalid fusion parameter: " + key)
    if not isinstance(f["imu_xyz_m"], list) or len(f["imu_xyz_m"]) != 3 or not all(finite(v) for v in f["imu_xyz_m"]):
        raise ValueError("Invalid IMU translation")
    if f["history_s"] > 3.0:
        raise ValueError("Replay history is limited to 3 seconds")
    return c
