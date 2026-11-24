from types import SimpleNamespace
import numpy as np
import pytest

from t701_ht_ros2.engine import Engine, MotionError


class Model:
    names = ("left_joint1", "left_joint2")
    sides = ("left",)
    count = 2
    lower, upper = np.full(2, -2.), np.full(2, 2.)
    velocity, acceleration = np.ones(2), np.full(2, 2.)
    def validate(self, q):
        assert np.all(np.isfinite(q))
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("limits")
    def collision(self, start, target):
        return {"pair": ("arm", "body")} if max(target) > .5 else None


class Backend:
    def __init__(self):
        self.q, self.v = np.zeros(2), np.zeros(2)
        self.commands, self.brakes, self.closed, self.failed = [], 0, False, False
    def read(self):
        if self.failed:
            raise RuntimeError("stale feedback")
        return self.q.copy(), self.v.copy(), np.zeros(2)
    def command(self, q, v):
        self.q, self.v = q.copy(), v.copy()
        self.commands.append((q.copy(), v.copy()))
    def brake(self):
        self.brakes += 1
        self.v.fill(0.)
    def close(self):
        self.closed = True
    def gripper(self, side, value):
        pass


def advance(engine, token):
    start = engine._active["start"]
    duration = engine._active["plan"].duration
    for now in np.arange(start, start + duration + .2, .02):
        engine.tick(now)
        if engine.result(token) is not None:
            break
    return engine.result(token)


def test_disabled_does_not_command():
    backend = Backend()
    engine = Engine(Model(), backend)
    with pytest.raises(MotionError):
        engine.submit_target(Model.names, [.1, 0.])
    engine.tick()
    assert not backend.commands


def test_names_limits_collision_and_duration_rejected_without_motion():
    b = Backend();e = Engine(Model(), b, enabled=True)
    for names, q, duration in [(('wrong', 'name'), [.1, 0.], 3.),
                                (Model.names, [3., 0.], 3.),
                                (Model.names, [.6, 0.], 3.),
                                (Model.names, [.1, 0.], .001)]:
        with pytest.raises((ValueError, MotionError)):
            e.submit_target(names, q, duration)
    assert not b.commands


def test_action_completes_and_holds_endpoint():
    b = Backend();e = Engine(Model(), b, enabled=True)
    token = e.submit_target(('left_joint2', 'left_joint1'), [0., .1], 1.)
    result = advance(e, token)
    assert result['success']
    np.testing.assert_allclose(b.q, [.1, 0.])
    e.tick(e._last_tick + .02)
    np.testing.assert_allclose(b.q, [.1, 0.])


def test_active_goal_rejected_then_cancel_starts_home():
    b = Backend();e = Engine(Model(), b, enabled=True)
    token = e.submit_target(Model.names, [.2, 0.], 2.)
    with pytest.raises(MotionError):
        e.submit_target(Model.names, [.1, 0.])
    start = e._active['start']
    for now in np.arange(start, start + .5, .02):e.tick(now)
    assert e.cancel(token)
    assert e.result(token)['canceled']
    home = e.status['active_id'];assert home is not None
    assert advance(e, home)['success']
    np.testing.assert_allclose(b.q, 0., atol=1e-10)
    assert b.brakes and not e.status['enabled']


def test_stop_brakes_without_home_or_further_commands():
    b = Backend();e = Engine(Model(), b, enabled=True)
    token = e.submit_target(Model.names, [.2, 0.])
    e.stop();count = len(b.commands)
    e.tick()
    assert e.result(token)['canceled'] and b.brakes
    assert len(b.commands) == count


def test_invalid_feedback_brakes_without_automatic_return():
    b = Backend();e = Engine(Model(), b, enabled=True)
    token = e.submit_target(Model.names, [.1, 0.])
    b.failed = True;e.tick()
    assert not e.result(token)['success'] and b.brakes
    assert e.status['mode'] == 'fault'


def test_runtime_collision_holds_and_rejects_goal():
    b = Backend();model = Model();e = Engine(model, b, enabled=True)
    token = e.submit_target(model.names, [.1, 0.], 1.)
    model.collision = lambda start, target: {'pair': ('arm', 'body')}
    e.tick()
    assert not e.result(token)['success']
    assert e.status['mode'] == 'collision_hold'
    np.testing.assert_array_equal(b.commands[-1][0], np.zeros(2))


def test_zero_time_point_must_match_measured_position():
    b = Backend();e = Engine(Model(), b, enabled=True)
    with pytest.raises(ValueError):
        e.submit_trajectory(Model.names, [dict(time_from_start=0., positions=[.1,0.]),
                                         dict(time_from_start=2., positions=[.2,0.])])


def test_shutdown_after_explicit_stop_does_not_restart_motion():
    b = Backend();e = Engine(Model(), b, enabled=True)
    e.stop();e.shutdown()
    assert b.closed and not b.commands
