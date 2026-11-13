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


@pytest.mark.parametrize('accuracy', [.128, 1., 10.])
def test_large_receiver_speed_uncertainty_is_retained_not_rejected(accuracy):
    e = engine()
    e.config['gnss_speed_accuracy_mps'] = .01
    observation = fix(10.)
    observation['speed_accuracy_mps'] = accuracy
    e.update_fix(observation, 10.)
    assert e.gnss_usable and e.velocity_valid
    assert e.fix['speed_accuracy_mps'] == accuracy


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
    assert b.initialized and b.sample(14.0) is not None
    b.check_timeout(17.)
    assert b.initialized and b.sample(17.) is None
    assert b.gap_recoveries == 0


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


def test_ros_input_queue_processes_each_sample_and_accepts_heading_without_confirmation():
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
    assert d.engine.initialized and d.engine.gap_recoveries == 0
    # No per-boot heading-quality confirmation gate: a fixed RTK solution and
    # a usable magnetic heading are enough (2026-11-13).
    assert "quality_confirmed" not in c["imu"]
    e = Inputs(c)
    feed(e, 10.0)
    assert e.engine.imu["ready"]


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


def test_planar_output_excludes_unobserved_vertical_drift():
    """A tilted, standing robot must not report body velocity from the
    unobserved vertical axis (regression for the fusion velocity audit)."""
    R = exp_rotation(np.array([0.1, -0.1, 0.0]))
    e = initialized(R)
    for i in range(1, 1001):
        t = 10.2 + i * 0.01
        if i % 10 == 0:
            e.update_fix(fix(t), t)
        if i % 20 == 0:
            heading(e, t, R)
        # 0.1 m/s^2 vertical accelerometer bias after the initial
        # stationary calibration; GNSS keeps reporting zero velocity.
        e.raw(R.T @ np.array([0.0, 0.0, 9.90665]), [0.0, 0.0, 0.0], t)
    state = e.filter.state
    twist = e.output["twist"]
    assert np.linalg.norm(state.v[:2]) < 0.001
    assert math.hypot(twist["linear_x"], twist["linear_y"]) < 0.02
    assert e.output["velocity_accuracy_mps"] < 0.1


def test_zupt_fires_after_standstill_and_not_while_moving():
    standing = initialized()
    for i in range(1, 501):
        t = 10.2 + i * 0.01
        if i % 10 == 0:
            standing.update_fix(fix(t), t)
        if i % 20 == 0:
            heading(standing, t)
        standing.raw([0, 0, 9.80665], [0, 0, 0], t)
    assert standing.filter.state.zupt_count > 0
    assert np.linalg.norm(standing.filter.state.v) < 0.01

    driving = initialized()
    moving_fix = fix(10.205)
    moving_fix['vel_e_mps'] = .5
    driving.update_fix(moving_fix, 10.205)
    for i in range(1, 501):
        t = 10.2 + i * 0.01
        if i % 10 == 0:
            f = fix(t)
            f["vel_e_mps"] = 0.5
            driving.update_fix(f, t)
        if i % 20 == 0:
            heading(driving, t)
        driving.raw([0, 0, 9.80665], [0, 0, 0], t)
    assert driving.filter.state.zupt_count == 0
    assert abs(driving.filter.state.v[0] - 0.5) < 0.02

    turning = initialized()
    for i in range(1, 501):
        t = 10.2 + i * 0.01
        if i % 10 == 0:
            turning.update_fix(fix(t), t)
        turning.raw([0, 0, 9.80665], [0, 0, 0.2], t)
    assert turning.filter.state.zupt_count == 0

    lost = initialized()
    lost.invalidate_gnss('disconnected')
    before = lost.filter.state.zupt_count
    for i in range(1, 51):
        lost.raw([0, 0, 9.80665], [0, 0, 0], 10.2+i*.01)
    assert lost.filter.state.zupt_count == before


def test_zupt_is_recomputed_for_delayed_gnss_including_start_and_stop():
    import copy

    a = initialized()
    b = copy.deepcopy(a)
    observations = []
    for i in range(1, 351):
        t = round(10.2 + i * .01, 9)
        if i % 10 == 0 and i <= 300:
            f = fix(t)
            # Start moving, then stop. Late delivery must not leave old
            # standstill corrections on the moving portion of the timeline.
            f['vel_e_mps'] = .5 if 60 <= i < 180 else 0.
            a.update_fix(f, t)
            observations.append((i + 25, t, f))
        for due, at, f in observations:
            if due == i:
                b.update_fix(f, at)
        for e in (a, b):
            e.raw([0, 0, 9.90665], [0, 0, 0], t)
    assert a.filter.state.zupt_count > 0
    assert a.filter.state.zupt_count == b.filter.state.zupt_count
    for field in ('p', 'v', 'P', 'ba', 'R'):
        np.testing.assert_allclose(getattr(a.filter.state, field),
                                   getattr(b.filter.state, field), atol=1e-10)


def test_standstill_detection_survives_shorter_replay_history():
    e = initialized()
    e.options['stationary_s'] = 2.5  # Longer than the 2-second replay window.
    for i in range(1, 321):
        t = 10.2+i*.01
        if i % 10 == 0:
            e.update_fix(fix(t), t)
        e.raw([0, 0, 9.90665], [0, 0, 0], t)
        if i < 240:
            assert e.filter.state.zupt_count == 0
    assert e.filter.state.zupt_count > 0


def test_planar_covariance_matches_velocity_and_excludes_vertical_state():
    import copy

    e = initialized(exp_rotation(np.array([.2, -.1, .5])))
    e.options['imu_xyz_m'] = [.3, -.2, .1]
    s = e.filter.state
    s.v = np.array([.6, .3, 100.])
    s.P = np.diag(np.linspace(.001, .015, 15))
    s.P[5, 5] = 10000.
    w = np.array([.03, .04, .2])
    e._output(w)
    base = copy.deepcopy(s)
    J = np.zeros((2, 15))
    eps = 1e-6
    for j in range(15):
        values = []
        for sign in (1, -1):
            s = copy.deepcopy(base)
            delta = np.zeros(3); delta[j % 3] = sign*eps
            if j < 3: s.p += delta
            elif j < 6: s.v += delta
            elif j < 9: s.R = s.R @ exp_rotation(delta)
            elif j < 12: s.bg += delta
            else: s.ba += delta
            e.filter.state = s
            e._output(w)
            values.append([e.output['twist']['linear_x'], e.output['twist']['linear_y']])
        J[:, j] = (np.array(values[0])-values[1])/(2*eps)
    e.filter.state = base
    e.options['gyro_noise'] = 1e-12  # Isolate the state Jacobian covariance.
    e._output(w)
    np.testing.assert_allclose(e.output['velocity_covariance'], J @ base.P @ J.T,
                               rtol=1e-6, atol=1e-9)
    assert np.linalg.norm(J[:, 5]) == 0.
    expected = math.sqrt(np.linalg.eigvalsh(e.output['velocity_covariance'])[-1])
    assert e.output['velocity_accuracy_mps'] == pytest.approx(expected)


def test_imu_gap_uses_float_and_resumes_without_recalibration():
    e = initialized()
    original_filter = e.filter
    bias = e.filter.state.ba.copy()
    for i in range(1, 21):
        t = 10.2+i*.1
        observation = fix(t, x=.4*(t-10.2), fixed=False)
        observation['vel_e_mps'] = .4
        e.update_fix(observation, t)
        e.check_timeout(t)
        assert e.ready(t) and e.grade == 'float'
    assert e.output['map_pose']['x'] > .5
    assert e.output['twist']['linear_x'] > .3
    assert e.diagnostics(t)['imu_degraded']
    e.raw([0, 0, 9.80665], [0, 0, 0], t+.01)
    assert e.filter is original_filter and e.ready(t+.01)
    assert e.gap_recoveries == 0 and not e.imu_degraded
    np.testing.assert_allclose(e.filter.state.ba, bias, atol=.01)
    assert np.linalg.eigvalsh(e.filter.state.P).min() >= -1e-10


def test_longer_imu_gap_uses_course_when_moving_but_not_stationary():
    e = initialized()
    for i in range(1, 51):
        t=10.2+i*.1
        f=fix(t, x=.5*(t-10.2), fixed=False)
        f['vel_e_mps'] = .5
        f['vel_n_mps'] = .1
        e.update_fix(f,t)
        e.check_timeout(t)
        assert e.ready(t)
    assert e.diagnostics(t)['heading_source'] == 'gnss_course'
    assert e.local_yaw > .01
    # Delayed observations still replay through the coast without clearing state.
    assert e.filter.insert(t-.2, 'heading', (.1, .01))
    assert np.linalg.eigvalsh(e.filter.state.P).min() >= -1e-10
    stationary = initialized()
    stationary.update_fix(fix(14., fixed=False), 14.)
    stationary.check_timeout(14.)
    assert stationary.initialized and stationary.sample(14.) is None
    heading(stationary, 14.01)
    stationary.raw([0, 0, 9.80665], [0, 0, 0], 14.01)
    assert stationary.ready(14.01) and stationary.gap_recoveries == 0


def test_stale_inertial_packet_does_not_clear_initialized_estimator():
    from types import SimpleNamespace as S
    from test_inputs import header
    from localization_outdoor.inputs import Inputs
    d=Inputs(engine().config)
    d.engine=initialized()
    raw=S(header=header(5.), orientation_covariance=[-1.]*9,
          linear_acceleration=S(x=0.,y=0.,z=9.80665),
          angular_velocity=S(x=0.,y=0.,z=0.))
    d.put('inertial', raw, 10.3, 10.3)
    d.step(10.3)
    assert d.engine.initialized and d.engine.gap_recoveries == 0
    assert d.engine.dropped_samples == 1


def test_invalid_imu_packet_does_not_refresh_stream_or_reset_filter():
    e=initialized()
    before=e.last_raw_at
    e.raw([float('nan'),0,0],[0,0,0],10.3)
    assert e.initialized and e.last_raw_at == before and e.dropped_samples == 1
    e.update_fix(fix(11.,fixed=False),11.)
    e.check_timeout(11.)
    assert e.ready(11.) and e.gap_recoveries == 0
