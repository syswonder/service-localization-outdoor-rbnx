"""A failed IMU replay invalidates localization instead of guessing motion."""
import copy
from types import SimpleNamespace as S
from unittest.mock import Mock

import numpy as np
import pytest

from localization_outdoor.eskf import InertialContinuityError
from test_fusion import initialized


def test_long_unobserved_interval_is_not_silently_coasted():
    e = initialized()
    f = e.filter
    stamp = f.state.t
    with pytest.raises(InertialContinuityError):
        f.insert(stamp + 1., 'inertial', (np.array([0., 0., 9.80665]), np.zeros(3)))
    assert f.state.t == stamp
    assert not f.state.coasting
    # A later valid sample must work against the pre-failure history.
    assert f.insert(stamp + .01, 'inertial', (np.array([0., 0., 9.80665]), np.zeros(3)))


def test_delayed_replay_rolls_back_all_history_on_failure(monkeypatch):
    f = initialized().filter
    state, events, checkpoints = copy.deepcopy((f.state, f.events, f.checkpoints))
    original = f.apply

    def broken(t, kind, data):
        original(t, kind, data)
        if kind == 'heading':
            raise InertialContinuityError('synthetic replay gap')

    monkeypatch.setattr(f, 'apply', broken)
    with pytest.raises(InertialContinuityError):
        f.insert(f.state.t - .01, 'heading', (0., .1))
    assert f.state.t == state.t
    np.testing.assert_array_equal(f.state.P, state.P)
    assert len(f.events) == len(events)
    assert len(f.checkpoints) == len(checkpoints)
    # Existing pre-failure observations, including headings, are retained exactly.
    assert [r[:2] for r in f.events] == [r[:2] for r in events]


def test_expected_estimator_gap_keeps_node_alive_and_publishes_invalid_status():
    pytest.importorskip('rclpy')
    from localization_outdoor.node import Localizer
    e = initialized()
    n = S(inputs=S(engine=e, imu_health=Mock(), imu_stamps=lambda: {},
                   step=Mock(side_effect=InertialContinuityError('gap'))),
          config={'require_map_reference': False}, active=True,
          get_logger=Mock(), odom=S(topic_name='/synthetic/odom'),
          count_publishers=lambda *_: 1, conflict=False,
          status_pub=Mock(), snapshot=lambda: {'ready': e.ready(10.3)})
    Localizer.tick(n)
    assert not e.initialized and e.output is None
    assert e.fault == 'inertial_continuity_lost'
    assert 'false' in n.status_pub.publish.call_args.args[0].data
