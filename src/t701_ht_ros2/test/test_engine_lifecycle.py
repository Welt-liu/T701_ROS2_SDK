import threading
import time
from unittest.mock import patch

import numpy as np
import pytest

from test_engine import Backend, Model, advance
from t701_ht_ros2.engine import Engine, MotionError


class GripperBackend(Backend):
    def __init__(self):
        super().__init__()
        self.settled = False
    def grippers_settled(self):
        return self.settled


def test_gripper_home_retains_token_and_excludes_all_normal_commands():
    backend = GripperBackend();engine = Engine(Model(), backend, enabled=True)
    token = engine.home()
    start = engine._active['start'];duration = engine._active['plan'].duration
    for now in np.arange(start, start + duration + .2, .02):engine.tick(now)
    assert engine.status['mode'] == 'gripper_homing'
    assert engine.status['active_id'] == token and engine.result(token) is None
    with pytest.raises(MotionError):engine.submit_target(Model.names, [.1, 0.])
    with pytest.raises(MotionError):engine.set_enabled(True)
    with pytest.raises(MotionError):engine.gripper('left', .5)
    engine.stop()
    assert engine.result(token)['canceled']
    assert backend.brakes


def test_home_brakes_only_after_grippers_confirmed():
    backend = GripperBackend();engine = Engine(Model(), backend, enabled=True)
    token = engine.home();start = engine._active['start'];duration = engine._active['plan'].duration
    for now in np.arange(start, start + duration + .2, .02):engine.tick(now)
    assert engine.result(token) is None and backend.brakes == 0
    backend.settled = True;engine.tick(engine._last_tick + .02)
    assert engine.result(token)['success'] and backend.brakes


def test_stop_preempts_collision_preprocessing_without_reenabling():
    backend = Backend();model = Model();engine = Engine(model, backend, enabled=True)
    started = threading.Event();errors = []
    def slow_collision(start, target):
        started.set();time.sleep(.01);return None
    model.collision = slow_collision
    def plan():
        try:engine.submit_target(model.names, [.1, 0.], 15.)
        except MotionError as error:errors.append(error)
    thread = threading.Thread(target=plan);thread.start();assert started.wait(2)
    before = time.monotonic();engine.stop();latency = time.monotonic() - before
    thread.join(3)
    assert latency < .1 and not thread.is_alive() and errors
    assert not engine.status['enabled'] and not backend.commands


def test_stop_during_home_start_cannot_restart_control():
    backend = Backend();engine = Engine(Model(), backend, enabled=True)
    original = engine.state;calls = 0
    def state_and_stop():
        nonlocal calls
        result = original();calls += 1
        if calls == 1:engine.stop()
        return result
    with patch.object(engine, 'state', side_effect=state_and_stop):
        with pytest.raises(MotionError):engine.home()
    assert not engine.status['enabled'] and backend.brakes and not backend.commands


def test_home_does_not_resume_old_qp_hold_while_planning():
    backend = Backend();model = Model();engine = Engine(model, backend, enabled=True)
    backend.q = np.array([.06, 0.])
    measured = backend.q.copy()
    engine._hold = np.array([.10, 0.])
    original = model.collision
    planning_tick = False
    def collision_with_idle_tick(start, target):
        nonlocal planning_tick
        if engine._planning and not planning_tick:
            planning_tick = True
            engine.tick()
        return original(start, target)
    model.collision = collision_with_idle_tick
    token = engine.home()
    assert planning_tick and engine.status['mode'] == 'homing'
    np.testing.assert_allclose(backend.commands[0][0], measured)
    assert advance(engine, token)['success']
    assert backend.brakes and not engine.status['enabled']
