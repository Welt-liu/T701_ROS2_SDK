"""Simulation and explicit hardware backends for the independent ROS 2 SDK.

The hardware extension is imported only when SDKBackend is constructed. Importing
this module, model display, and simulation never enumerate or open serial ports.
The motion engine owns position, velocity, acceleration and collision checks;
these backends additionally validate motor feedback and every queued frame.
"""
from __future__ import annotations

import copy
import importlib
from pathlib import Path
import threading
import time

import numpy as np

from .serial_adapter import prepared_config, read_yaml


KP = np.array([30., 50., 60., 25., 15., 10., 10.])
KD = np.array([3., 5., 6., 2.5, 1.5, 1., 1.])
GRIPPER_KP, GRIPPER_KD = 5., .5
FEEDBACK_MAX_AGE = .30
MIN_SEND_INTERVAL = .0052  # SDK commands are limited to 200 Hz.


class BackendError(RuntimeError):
    """Invalid, stale, disconnected or faulted hardware feedback/commands."""


def _vector(values, count, label):
    values = np.asarray(values, dtype=float)
    if values.shape != (count,) or not np.all(np.isfinite(values)):
        raise BackendError(f"{label} must contain {count} finite values")
    return values


def _sides(model):
    sides = tuple(model.sides)
    if not sides or len(set(sides)) != len(sides) or not set(sides) <= {"left", "right"}:
        raise BackendError("Select left, right or both arms")
    return sides


def _validate(model, q, velocity):
    count = 7 * len(model.sides)
    q, velocity = _vector(q, count, "Position"), _vector(velocity, count, "Velocity")
    model.validate(q)
    configured = _vector(model.velocity, count, "Velocity limits")
    if np.any(abs(velocity) > configured + 1e-8):
        raise BackendError("Velocity command exceeds configured motor limits")
    return q, velocity


class SimBackend:
    """Ideal kinematic follower for ROS interfaces, TF and collision demos.

    This is not a physics simulation. It applies validated commands exactly and
    requires no motor SDK or communication board.
    """

    def __init__(self, model):
        self.model, self.sides = model, _sides(model)
        self.count = 7 * len(self.sides)
        self._q, self._v = np.zeros(self.count), np.zeros(self.count)
        self._grippers = {side: 0. for side in self.sides}
        self.braked, self.closed = True, False
        self._lock = threading.RLock()

    def read(self):
        with self._lock:
            if self.closed:
                raise BackendError("Simulation backend is closed")
            return self._q.copy(), self._v.copy(), np.zeros(self.count)

    def command(self, q, velocity):
        with self._lock:
            if self.closed:
                raise BackendError("Simulation backend is closed")
            q, velocity = _validate(self.model, q, velocity)
            self._q, self._v = q.copy(), velocity.copy()
            self.braked = False

    def gripper(self, side, value):
        with self._lock:
            if self.closed or side not in self.sides or not np.isfinite(value):
                raise BackendError("Invalid gripper side or position")
            # The actual limits are checked by Engine; these are the SDK 2.0 limits.
            if not -.2 <= float(value) <= 2.2:
                raise BackendError("Gripper position exceeds SDK limits")
            self._grippers[side] = float(value)

    def gripper_positions(self):
        with self._lock:
            if self.closed:
                raise BackendError("Simulation backend is closed")
            return dict(self._grippers)

    def grippers_settled(self, tolerance=.05):
        with self._lock:
            if self.closed:
                raise BackendError("Simulation backend is closed")
            return True  # Ideal gripper applies the selected target immediately.

    def brake(self):
        with self._lock:
            self._v.fill(0.)
            self.braked = True

    def close(self):
        with self._lock:
            if not self.closed:
                self.brake()
                self.closed = True


def combined_motor_config(description_dir, sides):
    """Create only the selected CAN channels, with unique motor names."""
    description_dir = Path(description_dir)
    sides = tuple(sides)
    if not sides or len(set(sides)) != len(sides) or not set(sides) <= {"left", "right"}:
        raise BackendError("Invalid selected arms")
    result, gripper_limits, used_can = None, {}, set()
    for side in sides:
        config_path = description_dir / "config" / f"T701-HT_{side}.yaml"
        config = read_yaml(config_path)
        parameters = config["robot"]
        motor_path = (config_path.parent / parameters["param_file"]).resolve(strict=True)
        motor_config = read_yaml(motor_path)
        native = motor_config["robot"]
        if int(native["CANboard_num"]) != 1:
            raise BackendError("The current T701-HT ROS 2 backend supports one CAN board")
        board = native["CANboard"]["No_1_CANboard"]
        if int(board["CANport_num"]) != 1 or len(board["CANport"]) != 1:
            raise BackendError(f"{side} source configuration must select one CAN channel")
        port = copy.deepcopy(next(iter(board["CANport"].values())))
        can = port["can_port"]
        if type(can) is not int or not 1 <= can <= 7 or can in used_can:
            raise BackendError("Arm CAN channels must be distinct CAN1..7")
        used_can.add(can)
        motors = list(port["motor"].values())
        if int(port["motor_num"]) != 8 or sorted(int(m["id"]) for m in motors) != list(range(1, 9)):
            raise BackendError(f"{side} requires seven joints plus a gripper, IDs 1..8")
        for motor in motors:
            motor["name"] = f"ros_{side}_{int(motor['id'])}"
        if result is None:
            result = copy.deepcopy(motor_config)
            result["robot"]["CANboard"] = {
                "No_1_CANboard": {"CANport_num": len(sides), "CANport": {}}
            }
        else:
            for key in ("Serial_Type", "Seial_baudrate", "board_special_flag"):
                if result["robot"].get(key) != native.get(key):
                    raise BackendError(f"Arm communication configurations disagree: {key}")
        target_ports = result["robot"]["CANboard"]["No_1_CANboard"]["CANport"]
        target_ports[f"CANport_{len(target_ports) + 1}"] = port
        grip = parameters["gripper_limits"]
        lower, upper = float(grip["lower"]), float(grip["upper"])
        if not np.all(np.isfinite([lower, upper])) or not lower <= 0. <= upper or lower >= upper:
            raise BackendError(f"Invalid {side} gripper limits")
        gripper_limits[side] = (lower, upper)
    root = result["robot"]
    root["name"] = root["robot_name"] = "t701_ht_ros2"
    root["CANboard_num"] = 1
    root["motor_cmd_rate_limit_hz"] = 200
    # The native false branch resets motors at destruction; true uses damping brake.
    root["exit_motor_brake_flag"] = True
    root.pop("param_file", None)
    return result, gripper_limits


class SDKBackend:
    """Single native SDK object for selected arm(s); never controls unseen arms.

    Constructing this object opens the native SDK. Use only in explicitly selected
    hardware mode. The engine starts disabled and must gate every motion command.
    """

    def __init__(self, model, description_dir, *, native_factory=None):
        self.model, self.sides = model, _sides(model)
        self.count, self.description_dir = 7 * len(self.sides), Path(description_dir)
        self.robot, self.motors = None, {}
        self.closed, self.braked = False, True
        self._last_send = -float("inf")
        self._last_query = -float("inf")
        self._last_command = time.monotonic()
        self._lock = threading.RLock()
        config, self.gripper_limits = combined_motor_config(self.description_dir, self.sides)
        self._gripper_targets = {}
        self._gripper_commands = {}
        if native_factory is None:
            try:
                native_factory = importlib.import_module("hightorque_robot").Robot
            except ImportError as error:
                raise BackendError(
                    "Hardware mode requires the bundled hightorque_robot wheel for Python 3.10; "
                    "simulation does not require it"
                ) from error
        try:
            with prepared_config(config, self.description_dir / "config" / "serial_binding.yaml") as (path, rows):
                self.serial_mapping = rows
                self.robot = native_factory(str(path))
            motors = list(self.robot.get_motors())
            by_name = {motor.get_motor_name(): motor for motor in motors}
            expected = {f"ros_{side}_{i}" for side in self.sides for i in range(1, 9)}
            if len(motors) != len(expected) or len(by_name) != len(expected) or set(by_name) != expected:
                raise BackendError("SDK motor count/names do not match selected CAN channels")
            self.motors = {side: [by_name[f"ros_{side}_{i}"] for i in range(1, 9)]
                           for side in self.sides}
            for _ in range(5):
                self._query()
                time.sleep(.02)
            _, _, _, grippers = self._feedback()
            self._gripper_targets = grippers.copy()
            self._gripper_commands = grippers.copy()
        except BaseException:
            if self.robot is not None:
                try:
                    self.brake()
                except Exception:
                    pass
            self.closed = True
            raise

    def _flush(self):
        delay = self._last_send + MIN_SEND_INTERVAL - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        self.robot.motor_send_cmd()
        self._last_send = time.monotonic()

    def _query(self):
        # send_state_cmd queues each request; one send avoids per-motor bursts.
        for group in self.motors.values():
            for motor in group:
                motor.send_state_cmd()
        self._flush()
        self._last_query = time.monotonic()

    def _feedback(self):
        q, v, torque, grippers = [], [], [], {}
        now = time.time()
        for side in self.sides:
            for index, motor in enumerate(self.motors[side]):
                state, version = motor.get_current_motor_state(), motor.get_version()
                values = np.array([state.position, state.velocity, state.torque, state.time], dtype=float)
                age = now - values[3]
                if (not np.all(np.isfinite(values)) or int(state.fault) != 0
                        or not -.05 <= age <= FEEDBACK_MAX_AGE
                        or np.max(abs(values[:3])) >= 900.
                        or (int(version.major), int(version.minor), int(version.patch)) == (0, 0, 0)):
                    raise BackendError(f"{side} ID{index + 1} stale/invalid feedback: age={age:.3f}s fault={state.fault}")
                if index < 7:
                    q.append(float(values[0]));v.append(float(values[1]));torque.append(float(values[2]))
                else:
                    grippers[side] = float(values[0])
        q, v, torque = np.asarray(q), np.asarray(v), np.asarray(torque)
        self.model.validate(q)
        for side, value in grippers.items():
            lower, upper = self.gripper_limits[side]
            if not lower - .02 <= value <= upper + .02:
                raise BackendError(f"{side} gripper feedback exceeds limits")
        return q, v, torque, grippers

    def read(self):
        with self._lock:
            if self.closed:
                raise BackendError("Hardware backend is closed")
            if time.monotonic() - self._last_query >= .02:
                self._query()
            return self._feedback()[:3]

    def command(self, q, velocity):
        with self._lock:
            if self.closed:
                raise BackendError("Hardware backend is closed")
            q, velocity = _validate(self.model, q, velocity)
            actual, _, _, _ = self._feedback()
            gravity = _vector(self.model.gravity(actual), self.count, "Gravity feedforward")
            maximum = _vector(self.model.torque, self.count, "Maximum torque")
            gravity = np.clip(gravity, -maximum, maximum)
            now = time.monotonic()
            dt = max(0., min(now - self._last_command, .05))
            grippers = {}
            for side in self.sides:
                old, target = self._gripper_commands[side], self._gripper_targets[side]
                value = old + float(np.clip(target - old, -.5 * dt, .5 * dt))
                grippers[side] = value
            # All feedback, gravity and goals are valid before modifying any SDK buffer.
            for side_index, side in enumerate(self.sides):
                offset = side_index * 7
                for index, motor in enumerate(self.motors[side][:7]):
                    axis = offset + index
                    motor.pos_vel_tqe_kp_kd(float(q[axis]), float(velocity[axis]), float(gravity[axis]),
                                          float(KP[index]), float(KD[index]))
                self.motors[side][7].pos_vel_tqe_kp_kd(grippers[side], 0., 0., GRIPPER_KP, GRIPPER_KD)
            self._flush()
            self._gripper_commands = grippers
            self._last_command, self.braked = time.monotonic(), False

    def gripper(self, side, value):
        with self._lock:
            if self.closed or side not in self.sides or not np.isfinite(value):
                raise BackendError("Invalid gripper side or position")
            lower, upper = self.gripper_limits[side]
            if not lower <= float(value) <= upper:
                raise BackendError(f"{side} gripper command exceeds limits")
            self._gripper_targets[side] = float(value)

    def gripper_positions(self):
        with self._lock:
            if self.closed:
                raise BackendError("Hardware backend is closed")
            return dict(self._feedback()[3])

    def grippers_settled(self, tolerance=.05):
        with self._lock:
            if self.closed:
                raise BackendError("Hardware backend is closed")
            tolerance = float(tolerance)
            if not np.isfinite(tolerance) or tolerance < 0.:
                raise ValueError("Gripper tolerance must be finite and nonnegative")
            actual = self._feedback()[3]
            return all(abs(actual[side] - self._gripper_targets[side]) <= tolerance
                       and abs(float(self.motors[side][7].get_current_motor_state().velocity)) <= .15
                       for side in self.sides)

    def brake(self):
        with self._lock:
            if self.robot is None:
                return
            # Even if feedback is unavailable, every known selected motor gets a brake request.
            selected = [motor for group in self.motors.values() for motor in group]
            if not selected:
                selected = [motor for motor in self.robot.get_motors()
                            if motor.get_motor_name().startswith(tuple(f"ros_{side}_" for side in self.sides))]
            errors = []
            for _ in range(3):
                queued = False
                for motor in selected:
                    try:
                        motor.brake();queued = True
                    except Exception as error:
                        errors.append(str(error))
                if queued:
                    try:
                        self._flush()
                    except Exception as error:
                        errors.append(str(error))
            self.braked = True
            if errors:
                raise BackendError("Damping brake failed: " + "; ".join(errors))

    def close(self):
        with self._lock:
            if not self.closed:
                try:
                    self.brake()
                finally:
                    self.closed = True
