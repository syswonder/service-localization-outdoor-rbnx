import math
from types import SimpleNamespace as S
import pytest
from localization_outdoor.configuration import validate
from localization_outdoor.inputs import Inputs


def config():
    return validate(
        {
            "origin": {"latitude": 30.0, "longitude": 120.0},
            "calibrated": True,
            "antenna_offset_confirmed": True,
            "feedback_confirmed": True,
            "gnss_stable_s": 0.1,
            "velocity_accuracy_mps": 0.02,
            "velocity_accuracy_confirmed": True,
            "imu": {
                "base_from_imu_rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                "mounting_confirmed": True,
                "reference_confirmed": True,
                "magnetic_declination_rad": 0.0,
                "quality_confirmed": True,
                "yaw_std_deg": 1.0,
            },
        }
    )


def test_loaded_map_reference_validation():
    bound = {**config(), 'map_id':'beijing-v1', 'map_version':'a'*16,
             'require_map_reference':True, 'origin':{'latitude':39.9042,'longitude':116.4074}}
    assert validate(bound)['map_id'] == 'beijing-v1'
    for bad in ({'map_version':None}, {'map_id':'../map'}):
        with pytest.raises(ValueError):
            validate({**bound, **bad})


def header(now, frame="imu"):
    return S(
        stamp=S(sec=int(now), nanosec=round((now - int(now)) * 1e9)), frame_id=frame
    )


def feed(data, t, epoch="120000.00", speed=0.0, receiver_accuracy=None):
    t = round(t, 9)
    fix = S(
        header=header(t, "gnss"),
        status=S(status=2),
        latitude=30.0,
        longitude=120.0,
        position_covariance_type=2,
        position_covariance=[0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.04],
    )
    rtk = {
        "receiver_quality": 4,
        "state": "RTK_FIXED",
        "fix_age_s": 0.0,
        "measurement_age_s": 0.0,
        "utc_time": epoch,
        "fix_stamp_s": t,
        "velocity": {"fresh": True, "age_s": 0.0, "utc_time": epoch, "stamp_s": t},
    }
    if receiver_accuracy is not None:
        rtk.update(epoch_source='UBX_iTOW_ms',measurement_id=round(t*1000))
        rtk['velocity'].update(measurement_id=round(t*1000),accuracy_source='UBX_NAV_PVT',accuracy_mps=receiver_accuracy)
    velocity = S(header=header(t, "gnss_enu"), twist=S(linear=S(x=speed, y=0.0)))
    imu = S(
        header=header(t),
        orientation=S(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=[0.0] * 9,
    )
    gyro = S(header=header(t), vector=S(x=0.0, y=0.0, z=0.0))
    for role, msg in (
        ("gnss_fix", fix),
        ("rtk_status", rtk),
        ("gnss_velocity", velocity),
        ("imu", imu),
        ("gyro", gyro),
        ("imu_status", {"valid": True, "serial_connected": True}),
    ):
        data.put(role, msg, t, t)
    data.step(t)


def test_valid_inputs_create_pose_without_external_odom():
    d = Inputs(config())
    feed(d, 10.0)
    feed(d, 10.2, "120000.20")
    assert d.engine.ready(10.2)
    sample = d.engine.sample(10.2)
    assert (
        sample["map_pose"]["x"] == pytest.approx(0.0)
        and sample["twist"]["linear_x"] == 0.0
    )


def test_no_uncertainty_invention_or_unverified_north():
    d = Inputs(config())
    feed(d, 10.0)
    d.values["gnss_fix"][0].position_covariance_type = 0
    d.step(10.1)
    assert not d.engine.ready(10.1) and "uncertainty_unknown" in d.reason
    c = config()
    c["velocity_accuracy_confirmed"] = False
    d = Inputs(c)
    feed(d, 10.0)
    assert not d.engine.ready(10.0) and "unverified" in d.reason
    c = config()
    c["imu"]["reference_confirmed"] = False
    d = Inputs(c)
    feed(d, 10.0)
    assert not d.engine.heading_ready(10.0)


def test_repeat_epochs_cannot_refresh_position_and_lost_angles_still_allow_measured_stop():
    d = Inputs(config())
    feed(d, 10.0)
    feed(d, 10.2, "120000.20")
    assert d.engine.ready(10.2)
    # New receiver velocity/gyro but no new attitude: stopping feedback continues.
    for i in range(1, 8):
        t = round(10.2 + i * 0.1, 9)
        old = d.values["imu"]
        feed(d, t, f"120000.{20 + i * 10:02d}")
        d.values["imu"] = old
        d.step(t)
    assert not d.engine.heading_ready(10.9)
    assert d.engine.feedback_ready(10.9)
    d.step(12.0)
    assert not d.engine.feedback_ready(12.0)


def test_bad_configuration_rejected():
    for change in (
        {"origin": None},
        {"velocity_accuracy_confirmed": True},
        {"imu": {"mounting_confirmed": True}},
        {"serial": {"port": "/dev/example"}},
    ):
        with pytest.raises(ValueError):
            validate({**{"origin": {"latitude": 30.0, "longitude": 120.0}}, **change})


def test_reported_pvt_accuracy_does_not_need_a_fabricated_deployment_bound():
    c=config();c['velocity_accuracy_confirmed']=False;c['velocity_accuracy_mps']=None
    d=Inputs(c)
    feed(d,10.,receiver_accuracy=.02)
    feed(d,10.2,receiver_accuracy=.02)
    assert d.engine.ready(10.2) and d.engine.fix['speed_accuracy_mps']==.02
