import math
import numpy as np
import pytest
from test_inputs import config
from localization_outdoor.fusion import FusionFeedback
from localization_outdoor.eskf import exp_rotation, yaw_jacobian


def engine():
    c = config()
    c["estimator_mode"] = "eskf"
    c["fusion"]["stationary_s"] = 0.1
    c["gnss_stable_s"] = 0.1
    return FusionFeedback(c["origin"], c)


def fix(t, x=0.0, fixed=True):
    return dict(
        latitude=30.0,
        longitude=120.0 + math.degrees(x / (6378137 * math.cos(math.radians(30)))),
        hacc_m=0.05 if fixed else 0.5,
        valid=True,
        differential=True,
        fixed=fixed,
        float=not fixed,
        gga_quality=4 if fixed else 5,
        vel_e_mps=0.0,
        vel_n_mps=0.0,
        speed_accuracy_mps=0.02,
        tow_ms=round(t * 1000),
    )


def heading(e, t, R=None):
    R = np.eye(3) if R is None else R
    e.update_imu(
        dict(
            ready=True,
            body_yaw_enu_rad=math.atan2(R[1, 0], R[0, 0]),
            body_rotation=R,
            angle_at=t,
            gyro_at=t,
        ),
        t,
    )


def initialized(R=None):
    e = engine()
    R = np.eye(3) if R is None else R
    for i in range(21):
        t = 10 + i * 0.01
        e.update_fix(fix(t), t)
        heading(e, t, R)
        e.raw(R.T @ [0, 0, 9.80665], [0, 0, 0], t)
    assert e.initialized
    return e


def test_static_tilt_gravity_and_stationary_restart():
    R = exp_rotation(np.array([0.2, -0.1, 0]))
    e = initialized(R)
    for i in range(1, 101):
        e.raw(R.T @ [0, 0, 9.80665], [0, 0, 0], 10.2 + i * 0.01)
    assert np.linalg.norm(e.filter.state.v) < 1e-8
    assert np.linalg.norm(e.filter.state.p[:2]) < 1e-8
    # Check the heading measurement model under tilt against actual rotations.
    eps = 1e-7
    yaw = math.atan2(R[1, 0], R[0, 0])
    measured = []
    for axis in np.eye(3):
        shifted = R @ exp_rotation(axis * eps)
        measured.append((math.atan2(shifted[1, 0], shifted[0, 0]) - yaw) / eps)
    np.testing.assert_allclose(yaw_jacobian(R), measured, atol=1e-7)
    d = engine()
    for i in range(20):
        t = 10 + i * 0.01
        d.update_fix(fix(t), t)
        heading(d, t)
        d.raw([0, 0, 9.80665], [0, 0, 0.1 if i % 5 == 0 else 0], t)
    assert not d.initialized
    d.invalidate_gnss("lost")
    assert d.sample(11.0) is None


def test_acceleration_inertial_growth_and_fixed_reset_preserves_bias_odom():
    e = initialized()
    e.invalidate_gnss("lost")
    before = e.filter.state.P.copy()
    for i in range(1, 101):
        e.raw([1, 0, 9.80665], [0, 0, 0], 10.2 + i * 0.01)
    assert e.grade == "inertial"
    assert e.filter.state.v[0] == pytest.approx(0.99, abs=0.015)
    assert e.filter.state.p[0] == pytest.approx(0.49, abs=0.015)
    assert e.filter.state.P[0, 0] > before[0, 0]
    local = e.local_p.copy()
    bias = e.filter.state.ba.copy()
    e.update_fix(fix(11.2, x=100), 11.2)
    e.raw([1, 0, 9.80665], [0, 0, 0], 11.21)
    assert e.recovery_id == 1 and e.grade == "fixed"
    assert e.filter.state.p[0] > 99
    assert np.linalg.norm(e.local_p - local) < 0.05
    np.testing.assert_allclose(e.filter.state.ba, bias)
    e.options["max_horizontal_2sigma_m"] = 0.001
    assert e.sample(11.21) is not None and not e.ready(11.21)


def test_delayed_gnss_replay_equivalence_duplicate_and_expiration():
    a, b = initialized(), initialized()
    a.invalidate_gnss("lost")
    b.invalidate_gnss("lost")
    for i in range(1, 261):
        t = 10.2 + i * 0.01
        a.raw([0.2, 0, 9.80665], [0, 0, 0], t)
        b.raw([0.2, 0, 9.80665], [0, 0, 0], t)
        if i == 230:
            a.update_fix(fix(t, x=2), t)
    b.update_fix(fix(12.5, x=2), 12.5)
    a.raw([0.2, 0, 9.80665], [0, 0, 0], 12.81)
    b.raw([0.2, 0, 9.80665], [0, 0, 0], 12.81)
    np.testing.assert_allclose(a.filter.state.p, b.filter.state.p, atol=1e-10)
    np.testing.assert_allclose(a.filter.state.P, b.filter.state.P, atol=1e-10)
    before = b.filter.state.p.copy()
    b.raw([99, 0, 9.80665], [0, 0, 0], 12.81)
    np.testing.assert_array_equal(b.filter.state.p, before)
    assert b.dropped_samples == 1
    assert not b.filter.insert(1.0, "gnss", ([0, 0], [0, 0], 0.1, 0.1, False))
    b.check_timeout(14.0)
    assert not b.initialized and b.sample(14.0) is None


def test_float_weight_and_turning():
    e = initialized()
    e.invalidate_gnss("lost")
    e.update_fix(fix(10.205, x=10, fixed=False), 10.205)
    e.raw([0, 0, 9.80665], [0, 0, 0.2], 10.21)
    assert 0 < e.filter.state.p[0] < 10 and e.recovery_id == 0
    for i in range(2, 22):
        e.raw([0, 0, 9.80665], [0, 0, 0.2], 10.2 + i * 0.01)
    assert math.atan2(e.filter.state.R[1, 0], e.filter.state.R[0, 0]) == pytest.approx(
        0.04, abs=0.003
    )
    assert np.linalg.eigvalsh(e.filter.state.P).min() >= -1e-10


def test_ros_input_queue_processes_each_sample_and_rejects_unverified_heading():
    from types import SimpleNamespace as S
    from test_inputs import feed, header
    from localization_outdoor.inputs import Inputs

    c = engine().config
    d = Inputs(c)
    for i in range(31):
        t = 10 + i * 0.01
        if i % 2 == 0:
            feed(d, t, f"120000.{i:02d}")
        raw = S(
            header=header(t),
            orientation_covariance=[-1.0] + [0.0] * 8,
            linear_acceleration=S(x=0.0, y=0.0, z=9.80665),
            angular_velocity=S(x=0.0, y=0.0, z=0.0),
        )
        d.put("inertial", raw, t, t)
        d.step(t)
    assert d.engine.initialized
    for i in range(1, 11):
        t = 10.3 + i * 0.01
        raw.header = header(t)
        # A real ROS delivery owns its message; emulate that by copying here.
        import copy

        d.put("inertial", copy.deepcopy(raw), t, t)
    d.step(10.4)
    assert not d.inertial_queue
    assert sum(kind == "inertial" for _, kind, _ in d.engine.filter.events) >= 10
    assert d.engine.output["stamp"] == pytest.approx(10.4)
    d.put(
        "imu_status",
        {"generation": "new", "serial_connected": True, "valid": True},
        10.4,
        10.4,
    )
    d.put(
        "imu_status",
        {"generation": "changed", "serial_connected": True, "valid": True},
        10.41,
        10.41,
    )
    assert not d.engine.initialized
    c["imu"]["quality_confirmed"] = False
    e = Inputs(c)
    feed(e, 10.0)
    assert not e.engine.imu["ready"]


def test_scene_timers_reset_with_each_new_fixed_measurement():
    e = initialized()
    assert e.diagnostics(10.2)["fusion_elapsed_s"] == pytest.approx(0.0)
    e.invalidate_gnss("lost")
    e.raw([0, 0, 9.80665], [0, 0, 0], 10.21)
    e.raw([0, 0, 9.80665], [0, 0, 0], 10.31)
    d = e.diagnostics(10.31)
    assert d["inertial"] and d["inertial_elapsed_s"] == pytest.approx(0.1)
    assert d["rtk_grade"] == "unavailable"
    assert d["fusion_elapsed_s"] == pytest.approx(0.11)
    e.update_fix(fix(10.32, fixed=False), 10.32)
    e.raw([0, 0, 9.80665], [0, 0, 0], 10.33)
    assert not e.diagnostics(10.33)["inertial"]
    assert e.diagnostics(10.33)["rtk_grade"] == "float"
    assert e.diagnostics(10.33)["fusion_elapsed_s"] == pytest.approx(0.13)
    e.update_fix(fix(10.34), 10.34)
    e.raw([0, 0, 9.80665], [0, 0, 0], 10.35)
    assert e.diagnostics(10.35)["fusion_elapsed_s"] == pytest.approx(0.01)
    assert e.diagnostics(10.35)["rtk_grade"] == "fixed"


def test_partial_gnss_delivery_does_not_invalidate_recent_fix():
    import copy
    from test_inputs import feed
    from localization_outdoor.inputs import Inputs

    d = Inputs(engine().config)
    feed(d, 10.0)
    stable = d.engine.stable_since
    rtk = copy.deepcopy(d.values["rtk_status"][0])
    rtk["fix_stamp_s"] = 10.1
    rtk["velocity"]["stamp_s"] = 10.1
    d.put("rtk_status", rtk, 10.1, 10.1)
    d.step(10.1)
    assert d.engine.gnss_usable and d.engine.stable_since == stable
    rtk["receiver_quality"] = 0
    rtk["state"] = "NO_FIX"
    d.put("rtk_status", rtk, 10.11, 10.11)
    d.step(10.11)
    assert not d.engine.gnss_usable


def test_stationary_initialization_accepts_gnss_ahead_of_queued_imu():
    e = engine()
    for i in range(31):
        t = 10 + i * 0.01
        e.update_fix(fix(t + 0.02), t + 0.02)
        heading(e, t + 0.01)
        e.raw([0, 0, 9.80665], [0, 0, 0], t)
    assert e.initialized and e.grade == "fixed"


def test_no_raw_backlog_before_map_binding():
    from localization_outdoor.inputs import Inputs
    from types import SimpleNamespace as S
    from test_inputs import header

    c = engine().config
    c["require_map_reference"] = True
    d = Inputs(c)
    for i in range(600):
        t = 10 + i * 0.01
        d.put("inertial", S(header=header(t)), t, t)
    assert not d.inertial_queue and d.engine.fault is None
