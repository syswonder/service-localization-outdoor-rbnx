"""No device paths, NTRIP, vendor decoder or business map dependency."""

import math
import re
import numpy as np

DEFAULTS = {
    "sensor_timeout_s": 1.0,
    "gnss_stable_s": 2.0,
    "gnss_fixed_hacc_m": 3.0,
    "gnss_float_hacc_m": 3.0,
    "gnss_speed_accuracy_mps": 0.1,
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
        "quality_confirmed": False,
        "yaw_std_deg": None,
        "timeout_s": 0.5,
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
        "quality_confirmed",
        "course_heading",
    ):
        if type(imu[key]) is not bool:
            raise ValueError("IMU confirmation must be boolean")
    for key in ("magnetic_declination_rad", "yaw_std_deg"):
        if imu[key] is not None and not finite(imu[key]):
            raise ValueError("Invalid " + key)
    if not finite(imu["timeout_s"]) or not 0 < imu["timeout_s"] <= 2:
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
    if imu["quality_confirmed"] and (
        imu["yaw_std_deg"] is None or imu["yaw_std_deg"] <= 0
    ):
        raise ValueError("Measured yaw quality required")
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
    return c
