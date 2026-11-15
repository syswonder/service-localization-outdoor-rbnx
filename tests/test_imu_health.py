from localization_outdoor.imu_health import ImuHealth
from localization_outdoor.inputs import Inputs
from test_inputs import config


def test_gap_history_is_recorded_without_a_browser_and_does_not_claim_accuracy():
    health=ImuHealth(.15,3.)
    health.update(10.,{'raw':None,'attitude':None},100.)
    assert health.snapshot(10.,{'raw':None,'attitude':None},None)['state']=='waiting'
    stamps={'raw':10.,'attitude':10.}
    health.update(10.1,stamps,100.1)
    health.update(10.5,stamps,100.5)
    interrupted=health.snapshot(10.5,stamps,True)
    assert interrupted['state']=='interrupted' and interrupted['serial_connected'] is True
    assert interrupted['interruption_s'] == .35 or abs(interrupted['interruption_s']-.35)<1e-9
    health.update(11.,{'raw':11.,'attitude':11.},101.)
    resumed=health.snapshot(11.,{'raw':11.,'attitude':11.},True)
    assert resumed['state']=='recovered'
    assert [e['kind'] for e in resumed['events']]==['interrupted','recovered']
    assert abs(resumed['events'][-1]['duration_s']-.85)<1e-9
    assert 'attitude_valid' not in resumed
    assert health.snapshot(11.,{'raw':11.,'attitude':11.},True)==resumed


def test_serial_status_expires_to_unknown_and_diagnostics_do_not_change_estimator():
    inputs=Inputs(config())
    inputs.put('imu_status',{'serial_connected':False},10.,10.)
    before=(inputs.engine.initialized, inputs.engine.fault,inputs.engine.stable_since)
    assert inputs.imu_diagnostics(10.1)['serial_connected'] is False
    assert inputs.imu_diagnostics(13.1)['serial_connected'] is None
    assert before==(inputs.engine.initialized, inputs.engine.fault,inputs.engine.stable_since)


def test_history_is_bounded_and_attitude_only_gap_is_detected():
    health=ImuHealth(.15,3.)
    for i in range(30):
        t=10.+i
        health.update(t+.5,{'raw':t,'attitude':t},t+.5)
        health.update(t+.6,{'raw':t+.6,'attitude':t+.6},t+.6)
    assert len(health.events)==20
    health.update(50.,{'raw':50.,'attitude':46.},50.)
    snapshot=health.snapshot(50.,{'raw':50.,'attitude':46.},None)
    assert snapshot['state']=='interrupted' and snapshot['missing']==['attitude']
