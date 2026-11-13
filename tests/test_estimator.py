import pytest
from localization_outdoor.inputs import Inputs
from test_inputs import config, feed, header


@pytest.mark.parametrize('accuracy', [.128, 1., 10.])
def test_receiver_speed_uncertainty_is_not_a_readiness_threshold(accuracy):
    c = config()
    c['gnss_speed_accuracy_mps'] = .01  # Old deployment keys cannot re-enable it.
    data = Inputs(c)
    feed(data, 10., receiver_accuracy=accuracy)
    feed(data, 10.2, receiver_accuracy=accuracy)
    assert data.engine.ready(10.2)
    assert data.engine.sample(10.2)['velocity_accuracy_mps'] == accuracy


def test_receiver_velocity_integration_is_not_position_differencing():
    data = Inputs(config())
    feed(data, 10.0)
    # Hold GNSS position but provide measured eastward speed.
    for i in range(1, 6):
        t = round(10 + i * 0.1, 9)
        feed(data, t, f"120000.{i * 10:02d}", speed=1.0)
    assert data.engine.antenna_velocity == (1.0, 0.0)
    # The local path advances from receiver velocity, not GNSS finite differences.
    assert data.engine.output["odom_pose"]["x"] > 0.1


def test_day_rollover_and_frozen_status_do_not_make_new_epochs():
    data = Inputs(config())
    feed(data, 10.0, "235959.99")
    feed(data, 10.1, "000000.01")
    assert data.day == 1 and data.engine.fault is None
    before = data.engine.fix_at
    data.step(10.2)
    assert data.engine.fix_at == before
    data.step(13.2)
    assert not data.engine.ready(13.2)


def test_exact_stamp_association_rejects_mixed_messages():
    data = Inputs(config())
    feed(data, 10.0)
    data.values["rtk_status"][0]["velocity"]["stamp_s"] = 99.0
    data.step(10.1)
    assert data.reason == "gnss_stream_stale" and not data.engine.feedback_ready(10.1)


def test_independent_ros_angle_and_gyro_receive_stamps_can_initialize():
    data = Inputs(config())
    feed(data, 10.0)
    engine = data.engine
    engine.initialized = False
    engine.last_gyro_at = None
    observation = {
        "ready": True,
        "body_yaw_enu_rad": 0.0,
        "body_yaw_rate_rps": 0.0,
        "angle_at": 10.001,
        "gyro_at": 10.0,
    }
    engine.update_imu(observation, 10.002)
    assert engine.initialized and engine.feedback_ready(10.002)
