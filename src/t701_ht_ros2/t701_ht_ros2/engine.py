"""ROS-independent, serialized trajectory controller shared by mock and real backends.

All public joint vectors use the SDK's motor coordinates (radians). Model joint
signs are applied only by ArmModel, including the separate /joint_states output.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np
from scipy.interpolate import BPoly, PPoly


class MotionError(RuntimeError):
    pass


def vector(values, count, label):
    result = np.asarray(values, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} needs {count} finite values")
    return result.copy()


def extrema(poly, derivative=0):
    """Analytic per-axis extrema for one polynomial segment, including endpoints."""
    values = poly.derivative(derivative).c[:, 0, :]
    span = float(poly.x[-1] - poly.x[0])
    bounds = []
    for coefficients in values.T:
        roots = np.roots(np.polyder(coefficients))
        points = [0., span] + [float(root.real) for root in roots
                               if abs(root.imag) < 1e-8 and 0. < root.real < span]
        samples = np.polyval(coefficients, points)
        bounds.append((min(samples), max(samples)))
    return np.asarray(bounds).T


class Plan:
    """Piecewise quintics; validates derivatives and curved-path collision samples."""

    def __init__(self, model, start_q, start_v, points, canceled=lambda: False):
        self.parts = []
        self.duration = 0.
        q, v = start_q.copy(), start_v.copy()
        a = np.zeros(model.count)
        for point in points:
            if canceled():
                raise MotionError("planning canceled by a stop or another control request")
            end = float(point["time_from_start"])
            if not math.isfinite(end) or end <= self.duration:
                raise ValueError("trajectory times must be positive and strictly increasing")
            if end > 300.:
                raise ValueError("one goal is limited to 300 seconds")
            target = vector(point["positions"], model.count, "positions")
            target_v = vector(point.get("velocities", np.zeros(model.count)), model.count, "velocities")
            target_a = vector(point.get("accelerations", np.zeros(model.count)), model.count, "accelerations")
            model.validate(target)
            span = end - self.duration
            poly = PPoly.from_bernstein_basis(BPoly.from_derivatives(
                [0., span], [[q, v, a], [target, target_v, target_a]],
            ))
            low, high = extrema(poly)
            if np.any(low < model.lower - 1e-8) or np.any(high > model.upper + 1e-8):
                raise MotionError("interpolated trajectory exceeds joint limits")
            vl, vh = extrema(poly, 1)
            al, ah = extrema(poly, 2)
            peak_v = np.maximum(abs(vl), abs(vh))
            peak_a = np.maximum(abs(al), abs(ah))
            if np.any(peak_v > model.velocity + 1e-6):
                raise MotionError("trajectory exceeds joint velocity limits; increase duration")
            if np.any(peak_a > model.acceleration + 1e-6):
                raise MotionError("trajectory exceeds joint acceleration limits; increase duration")
            steps = max(1, int(np.ceil(span / .05)), int(np.ceil(max(peak_v) * span / .0125)))
            if steps > 12000:
                raise MotionError("trajectory segment needs too many collision samples")
            previous = q.copy()
            for sample in np.linspace(0., span, steps + 1):
                if canceled():
                    raise MotionError("planning canceled by a stop or another control request")
                current = poly(sample)
                hit = model.collision(previous, current)
                if hit is not None:
                    raise MotionError(f"trajectory collision: {hit['pair']}")
                previous = current
            self.parts.append((self.duration, end, poly))
            self.duration = end
            q, v, a = target, target_v, target_a
        if not self.parts:
            raise ValueError("trajectory must contain at least one point")
        if max(abs(v)) > 1e-8 or max(abs(a)) > 1e-8:
            raise ValueError("final velocity and acceleration must be zero")
        self.target = q.copy()

    def sample(self, elapsed):
        elapsed = float(np.clip(elapsed, 0., self.duration))
        for start, end, poly in self.parts:
            if elapsed <= end:
                t = elapsed - start
                return poly(t), poly(t, 1), poly(t, 2)
        raise AssertionError("trajectory time outside plan")


class Engine:
    """One command owner. Motion is never sent by subscriber or action threads."""

    def __init__(self, model, backend, sample_period=.02, enabled=False):
        self.model, self.backend = model, backend
        self.sample_period = float(sample_period)
        self._lock = threading.RLock()
        self._enabled = bool(enabled)
        self._active = None
        self._results = {}
        self._next_id = 0
        self._mode = "idle" if enabled else "disabled"
        self._last_error = ""
        self._collision_blocked = False
        self._tracking_since = None
        self._hold = None
        self._last_tick = None
        self._stopped = False
        self._ik_adapters = {}
        self._ik_solvers = {}
        self._planning = False
        self._epoch = 0
        q, _, _ = self.state()
        self.model.validate(q)
        hit = self.model.collision(q, q)
        if hit is not None:
            self.backend.brake()
            raise MotionError(f"initial configuration intersects collision geometry: {hit['pair']}")

    @property
    def status(self):
        with self._lock:
            return dict(enabled=self._enabled, mode=self._mode,
                        active_id=self._active["id"] if self._active else None,
                        last_error=self._last_error,
                        collision_blocked=self._collision_blocked)

    def state(self):
        with self._lock:
            q, v, effort = self.backend.read()
            return tuple(vector(x, self.model.count, label) for x, label in
                         ((q, "position feedback"), (v, "velocity feedback"), (effort, "effort feedback")))

    def result(self, token):
        with self._lock:
            result = self._results.get(token)
            return dict(result) if result is not None else None

    def _finish(self, success, error="", canceled=False):
        if self._active is not None:
            token = self._active["id"]
            self._results[token] = dict(success=success, error=error, canceled=canceled)
            if len(self._results) > 200:
                del self._results[min(self._results)]
        self._active = None
        self._tracking_since = None

    def _ordered(self, names, points):
        names = tuple(names)
        if len(names) != self.model.count or len(set(names)) != len(names) or set(names) != set(self.model.names):
            raise ValueError("joint names must contain every selected motor joint exactly once")
        order = [names.index(name) for name in self.model.names]
        if not points or len(points) > 2000:
            raise ValueError("expected 1 to 2000 trajectory points")
        result = []
        for point in points:
            row = dict(point)
            for key in ("positions", "velocities", "accelerations"):
                if key in row:
                    row[key] = vector(row[key], self.model.count, key)[order]
            result.append(row)
        # ROS trajectories may include an explicit current-state point at time 0.
        if float(result[0]["time_from_start"]) == 0.:
            current = self.state()[0]
            if max(abs(result[0]["positions"] - current)) > .02:
                raise ValueError("time-zero point must match current feedback")
            result = result[1:]
        return result

    def submit_trajectory(self, names, points):
        with self._lock:
            if not self._enabled:
                raise MotionError("control is disabled; call /t701_ht/set_enabled first")
            if self._active is not None or self._planning:
                raise MotionError("another trajectory is active; cancel or wait for it")
            q, v, _ = self.state()
            self.model.validate(q)
            ordered = self._ordered(names, points)
            self._planning = True
            epoch = self._epoch
        try:
            # Geometry preprocessing may take seconds. The command owner remains
            # free to hold/brake, and stop invalidates this pending plan immediately.
            plan = Plan(self.model, q, v, ordered, canceled=lambda: self._epoch != epoch)
        except BaseException:
            with self._lock:
                if self._epoch == epoch:
                    self._planning = False
            raise
        with self._lock:
            if self._epoch != epoch or not self._enabled:
                raise MotionError("control changed while planning; goal not sent")
            self._planning = False
            actual = self.state()[0]
            if max(abs(actual - q)) > .02:
                raise MotionError("arm moved while planning; resubmit from current feedback")
            self._next_id += 1
            self._active = dict(id=self._next_id, plan=plan, start=time.monotonic())
            self._mode = "trajectory"
            self._last_error = ""
            self._collision_blocked = False
            self._tracking_since = None
            self._stopped = False
            return self._next_id

    def submit_target(self, names, positions, duration=3.):
        duration = float(duration)
        if not math.isfinite(duration) or duration <= 0.:
            raise ValueError("duration must be positive")
        return self.submit_trajectory(names, [dict(time_from_start=duration, positions=positions)])

    def submit_pose(self, side, transform):
        """Cartesian targets use the unchanged Python SDK 2.0 OSQP solver."""
        with self._lock:
            if not self._enabled or self._active is not None or self._planning:
                raise MotionError("enable control and finish the active motion first")
            if side not in self.model.sides:
                raise ValueError("target side is not selected")
            target = np.asarray(transform, dtype=float)
            if target.shape != (4, 4) or not np.all(np.isfinite(target)):
                raise ValueError("expected a finite 4x4 pose in base_link")
            if (not np.allclose(target[3], [0., 0., 0., 1.]) or
                    not np.allclose(target[:3, :3].T @ target[:3, :3], np.eye(3), atol=1e-5) or
                    not np.isclose(np.linalg.det(target[:3, :3]), 1., atol=1e-5)):
                raise ValueError("invalid rigid transform")
            from .qp_adapter import ArmQPAdapter
            from .qp_ik import PinocchioOSQPIK
            q, _, _ = self.state()
            adapter = self._ik_adapters.get(side)
            if adapter is None:
                adapter = ArmQPAdapter(self.model, side)
                self._ik_adapters[side] = adapter
                self._ik_solvers[side] = PinocchioOSQPIK(
                    adapter, max_joint_velocity=.5, max_joint_acceleration=2.,
                    max_linear_velocity=.10, max_angular_velocity=.5,
                    singularity_threshold=.02, singularity_damping=.01,
                )
            adapter.set_context(q)
            solver = self._ik_solvers[side]
            index = self.model.sides.index(side) * 7
            solver.reset(posture_reference=q[index:index + 7])
            self._next_id += 1
            self._active = dict(id=self._next_id, kind="qp_ik", side=side,
                                target=target.copy(), command=q.copy(), start=time.monotonic())
            self._mode = "qp_ik"
            self._last_error = ""
            self._collision_blocked = False
            self._tracking_since = None
            self._stopped = False
            return self._next_id

    def _tick_ik(self, active, actual, now):
        side = active["side"]
        index = self.model.sides.index(side) * 7
        adapter, solver = self._ik_adapters[side], self._ik_solvers[side]
        adapter.set_context(actual)
        command = active["command"].copy()
        try:
            result = solver.step(command[index:index + 7], active["target"], self.sample_period)
        except Exception as error:
            solver.reset()
            self._finish(False, str(error))
            self._last_error = str(error)
            self._mode = "ik_hold"
            self._hold = actual.copy()
            self.backend.command(actual, np.zeros(self.model.count))
            return
        command[index:index + 7] = result.joint_position
        velocity = np.zeros(self.model.count)
        velocity[index:index + 7] = result.joint_velocity
        hit = self.model.collision(actual, command)
        if hit is None:
            hit = self.model.collision(command, command + .10 * velocity)
        if hit is not None:
            solver.reset()
            self._collision_blocked = True
            self._finish(False, f"QP IK collision: {hit['pair']}")
            self._mode = "collision_hold"
            self._hold = actual.copy()
            self.backend.command(actual, np.zeros(self.model.count))
            return
        if max(abs(command - actual)) > .35:
            raise MotionError("QP IK tracking error exceeds 0.35 rad")
        self.backend.command(command, velocity)
        active["command"] = command
        measured = self.model.forward(actual)[side]
        from scipy.spatial.transform import Rotation
        measured_position_error = np.linalg.norm(active["target"][:3, 3] - measured[:3, 3])
        measured_rotation_error = Rotation.from_matrix(active["target"][:3, :3] @ measured[:3, :3].T).magnitude()
        measured_velocity = self.state()[1]
        if (result.position_error < .002 and result.rotation_error < .03 and
                measured_position_error < .002 and measured_rotation_error < .03 and
                max(abs(command - actual)) < .03 and max(abs(measured_velocity)) < .08 and
                max(abs(result.joint_velocity)) < .03):
            self._hold = command.copy()
            self._finish(True)
            self._mode = "idle"
        elif now - active["start"] > 20.:
            self._hold = actual.copy()
            self._finish(False, "QP IK did not converge within 20 seconds")
            self._last_error = "QP IK target was not reached"
            self._mode = "ik_hold"

    def _start_home(self):
        q, v, _ = self.state()
        self.model.validate(q)
        distance = abs(q)
        duration = max(3., float(np.max(1.875 * distance / np.minimum(self.model.velocity, .3))),
                       float(np.sqrt(np.max(6. * distance / self.model.acceleration))))
        for side in self.model.sides:
            self.backend.gripper(side, 0.)
        token = self.submit_target(self.model.names, np.zeros(self.model.count), duration)
        self._mode = "homing"
        return token

    def home(self):
        with self._lock:
            if not self._enabled or self._mode == "fault":
                raise MotionError("enable healthy control before requesting home")
            # The prior QP/trajectory endpoint can differ from measured arrival.
            # Keep the measured start while the return path is planned, rather
            # than letting an idle tick continue toward that old endpoint.
            q, _, _ = self.state()
            self.model.validate(q)
            if not self._enabled:
                raise MotionError("control stopped before home planning")
            self._hold = q.copy()
            self._epoch += 1
            self._planning = False
            self._finish(False, "replaced by home request", canceled=True)
        try:
            return self._start_home()
        except Exception as exc:
            with self._lock:
                self._emergency(str(exc))
            raise

    def cancel(self, token, return_home=True):
        with self._lock:
            if self._active is None or self._active["id"] != token:
                return False
            self._finish(False, "canceled; returning home" if return_home else "canceled", canceled=True)
        if return_home:
            try:
                self.home()
            except Exception as exc:
                with self._lock:
                    self._emergency(str(exc))
        else:
            self.stop()
        return True

    def _emergency(self, reason):
        self._epoch += 1
        self._planning = False
        self._finish(False, reason)
        self._last_error = reason
        self._enabled = False
        self._mode = "fault"
        self._hold = None
        self.backend.brake()
        self._stopped = True

    def stop(self):
        with self._lock:
            self._epoch += 1
            self._planning = False
            self._finish(False, "emergency damping brake", canceled=True)
            self._enabled = False
            self._mode = "braked"
            self._hold = None
            self.backend.brake()
            self._stopped = True

    def set_enabled(self, enabled):
        with self._lock:
            if not enabled:
                self.stop()
                return
            if self._active is not None or self._planning:
                raise MotionError("cannot change enable state during an active trajectory")
            q, _, _ = self.state()
            self.model.validate(q)
            hit = self.model.collision(q, q)
            if hit is not None:
                raise MotionError(f"current configuration collision: {hit['pair']}")
            self._enabled = True
            self._mode = "idle"
            self._last_error = ""
            self._collision_blocked = False
            self._hold = q.copy()
            self._last_tick = None
            self._stopped = False

    def gripper(self, side, value):
        with self._lock:
            if not self._enabled:
                raise MotionError("control is disabled")
            if self._mode in ("homing", "gripper_homing"):
                raise MotionError("gripper goals are disabled during return home")
            self.state()  # do not send on invalid/stale feedback
            self.backend.gripper(side, value)

    def tick(self, now=None):
        with self._lock:
            now = time.monotonic() if now is None else float(now)
            try:
                q, v, _ = self.state()
                self.model.validate(q)
                if not self._enabled:
                    return
                if self._last_tick is not None and now - self._last_tick > .5:
                    raise MotionError("controller cycle stalled for over 0.5 seconds")
                self._last_tick = now
                if self._active is None:
                    target = q.copy() if self._hold is None else self._hold
                    hit = self.model.collision(q, target)
                    if hit is not None:
                        raise MotionError(f"hold path collision: {hit['pair']}")
                    self.backend.command(target, np.zeros(self.model.count))
                    self._hold = target.copy()
                    return
                active = self._active
                if active.get("kind") == "gripper_home":
                    target = active["target"]
                    hit = self.model.collision(q, target)
                    if hit is not None:
                        raise MotionError(f"home hold collision: {hit['pair']}")
                    self.backend.command(target, np.zeros(self.model.count))
                    if getattr(self.backend, "grippers_settled", lambda: True)():
                        self._finish(True)
                        self.backend.brake()
                        self._enabled = False
                        self._mode = "braked"
                        self._stopped = True
                    elif now > active["deadline"]:
                        raise MotionError("gripper home did not settle within 6 seconds")
                    return
                if active.get("kind") == "qp_ik":
                    self._tick_ik(active, q, now)
                    return
                plan = active["plan"]
                elapsed = now - active["start"]
                desired, velocity, acceleration = plan.sample(elapsed)
                hit = self.model.collision(q, desired)
                future, _, _ = plan.sample(min(elapsed + .10, plan.duration))
                if hit is None:
                    hit = self.model.collision(desired, future)
                if hit is not None:
                    self._collision_blocked = True
                    self._finish(False, f"motion blocked by collision: {hit['pair']}")
                    self._last_error = f"collision: {hit['pair']}"
                    self._mode = "collision_hold"
                    self._hold = q.copy()
                    self.backend.command(q, np.zeros(self.model.count))
                    return
                error = max(abs(desired - q))
                if error > .8:
                    raise MotionError(f"tracking error {error:.3f} rad exceeds 0.8 rad")
                if error > .35:
                    self._tracking_since = self._tracking_since or now
                    if now - self._tracking_since > .5:
                        raise MotionError(f"persistent tracking error {error:.3f} rad")
                else:
                    self._tracking_since = None
                self.backend.command(desired, velocity)
                if elapsed >= plan.duration:
                    if max(abs(q - plan.target)) <= .03 and max(abs(v)) <= .08:
                        homing = self._mode == "homing"
                        self._hold = plan.target.copy()
                        completed_token = active["id"]
                        if homing:
                            self._active = dict(id=completed_token, kind="gripper_home", target=plan.target.copy(), deadline=now + 6.)
                            self._mode = "gripper_homing"
                        else:
                            self._finish(True)
                            self._mode = "idle"
                    elif elapsed > plan.duration + 10.:
                        raise MotionError("trajectory did not settle within 10 seconds")
            except Exception as exc:
                self._emergency(str(exc))

    def shutdown(self, return_home=True):
        """Orderly SIGINT/SIGTERM returns home; fault/explicit stop never auto-restarts."""
        try:
            with self._lock:
                should_home = return_home and self._enabled and not self._stopped and self._mode != "fault"
            token = self.home() if should_home else None
            if token is not None:
                deadline = time.monotonic() + 40.
                while (self.result(token) is None or self._mode == "gripper_homing") and time.monotonic() < deadline:
                    self.tick()
                    time.sleep(self.sample_period)
        finally:
            self.stop()
            self.backend.close()
