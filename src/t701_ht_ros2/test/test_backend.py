"""Backend/serial regression with fake SDK only; no USB device is opened."""
from contextlib import contextmanager
import copy
from types import SimpleNamespace as NS

import numpy as np
import pytest
import yaml

from t701_ht_ros2 import backend
from t701_ht_ros2.serial_adapter import resolve_ports


class Model:
    def __init__(self, sides=("left", "right")):
        self.sides = sides
        self.names = tuple(f"{side}_joint{i}" for side in sides for i in range(1, 8))
        self.velocity = np.ones(len(self.names))
        self.torque = np.ones(len(self.names)) * 10.

    def validate(self, q):
        if q.shape != (len(self.names),) or not np.all(np.isfinite(q)) or np.any(abs(q) > 3.):
            raise ValueError("position limit")

    def gravity(self, q):
        return np.ones(len(q)) * 20.  # Must be clipped to motor torque limits.


class Clock:
    def __init__(self):
        self.now = 1000.

    def monotonic(self):
        return self.now

    def time(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class Motor:
    def __init__(self, name, index, clock):
        self.name, self.index, self.clock = name, index, clock
        self.pos, self.vel, self.fault = 0., 0., 0
        self.age, self.command = 0., None

    def get_motor_name(self):
        return self.name

    def get_motor_id(self):
        return self.index

    def get_version(self):
        return NS(major=4, minor=9, patch=1)

    def get_current_motor_state(self):
        return NS(position=self.pos, velocity=self.vel, torque=0.,
                  time=self.clock.time() - self.age, fault=self.fault)

    def send_state_cmd(self):
        self.command = ("read",)

    def pos_vel_tqe_kp_kd(self, *values):
        self.command = ("mit", *values)

    def brake(self):
        self.command = ("brake",)


class Native:
    def __init__(self, sides, clock):
        self.clock, self.frames = clock, []
        self.motors = [Motor(f"ros_{side}_{i}", i, clock) for side in sides for i in range(1, 9)]

    def get_motors(self):
        return list(reversed(self.motors))

    def motor_send_cmd(self):
        self.frames.append((self.clock.now, {m.name: m.command for m in self.motors}))


@pytest.fixture
def description(tmp_path):
    root = tmp_path / "description"
    (root / "config" / "motor_param").mkdir(parents=True)
    for side, can in (("left", 1), ("right", 2)):
        high = {"robot": {"param_file": f"motor_param/{side}.yaml",
                           "gripper_limits": {"lower": -.2, "upper": 2.2}}}
        native = {"robot": {"Serial_Type": "/dev/ttyACM", "Seial_baudrate": 4000000,
                            "board_special_flag": False, "CANboard_num": 1,
                            "CANboard": {"No_1_CANboard": {"CANport_num": 1, "CANport": {
                                "CANport_1": {"can_port": can, "serial_id": can, "motor_num": 8,
                                              "motor": {f"motor{i}": {"id": i, "name": "joint", "type": "4438_30"}
                                                        for i in range(1, 9)}}}}}}}
        (root / "config" / f"T701-HT_{side}.yaml").write_text(yaml.safe_dump(high))
        (root / "config" / "motor_param" / f"{side}.yaml").write_text(yaml.safe_dump(native))
    (root / "config" / "serial_binding.yaml").write_text('board_serial: "12345"\n')
    return root


@pytest.fixture
def fake_clock(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(backend.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(backend.time, "time", clock.time)
    monkeypatch.setattr(backend.time, "sleep", clock.sleep)
    return clock


def make_hardware(description, monkeypatch, clock, sides=("left", "right"), native=None):
    @contextmanager
    def prepared(config, bindings_path):
        yield description / "unused_native.yaml", [{"can_port": i + 1} for i in range(len(sides))]
    monkeypatch.setattr(backend, "prepared_config", prepared)
    native = native or Native(sides, clock)
    hardware = backend.SDKBackend(Model(sides), description, native_factory=lambda path: native)
    return hardware, native


def test_simulation_never_imports_native(monkeypatch):
    def no_native(name):
        raise AssertionError("native SDK import forbidden")
    monkeypatch.setattr(backend.importlib, "import_module", no_native)
    sim = backend.SimBackend(Model(("right",)))
    sim.command(np.ones(7) * .1, np.ones(7) * .2)
    q, v, torque = sim.read()
    np.testing.assert_allclose(q, .1)
    np.testing.assert_allclose(v, .2)
    np.testing.assert_allclose(torque, 0.)
    sim.gripper("right", .5)
    with pytest.raises(backend.BackendError):
        sim.gripper("left", .5)
    sim.brake()
    np.testing.assert_allclose(sim.read()[1], 0.)
    sim.close()
    with pytest.raises(backend.BackendError):
        sim.read()


@pytest.mark.parametrize("sides", [("left",), ("right",), ("left", "right")])
def test_selected_can_config_only(description, sides):
    original = (description / "config" / "motor_param" / f"{sides[0]}.yaml").read_bytes()
    config, grippers = backend.combined_motor_config(description, sides)
    ports = config["robot"]["CANboard"]["No_1_CANboard"]["CANport"]
    assert len(ports) == len(sides)
    assert [p["can_port"] for p in ports.values()] == [1 if side == "left" else 2 for side in sides]
    assert {m["name"] for p in ports.values() for m in p["motor"].values()} == {
        f"ros_{side}_{i}" for side in sides for i in range(1, 9)}
    assert config["robot"]["exit_motor_brake_flag"] is True
    assert set(grippers) == set(sides)
    assert (description / "config" / "motor_param" / f"{sides[0]}.yaml").read_bytes() == original


@pytest.mark.parametrize("sides", [("right",), ("left", "right")])
def test_sdk_init_read_only_mit_command_and_brake(description, monkeypatch, fake_clock, sides):
    hardware, native = make_hardware(description, monkeypatch, fake_clock, sides)
    assert len(native.frames) == 5
    assert all(set(commands.values()) == {("read",)} for _, commands in native.frames)
    hardware.command(np.full(7 * len(sides), .1), np.full(7 * len(sides), .2))
    commands = native.frames[-1][1]
    assert len(commands) == 8 * len(sides)
    for side in sides:
        for i in range(1, 8):
            command = commands[f"ros_{side}_{i}"]
            assert command[:4] == ("mit", .1, .2, 10.)
        assert commands[f"ros_{side}_8"][1] == 0.
    hardware.brake()
    assert all(set(frame.values()) == {("brake",)} for _, frame in native.frames[-3:])
    assert all(b[0] - a[0] >= backend.MIN_SEND_INTERVAL - 1e-10 for a, b in zip(native.frames, native.frames[1:]))
    hardware.close()


@pytest.mark.parametrize("failure", ["stale", "fault", "nan"])
def test_invalid_feedback_blocks_entire_frame(description, monkeypatch, fake_clock, failure):
    hardware, native = make_hardware(description, monkeypatch, fake_clock)
    before = len(native.frames)
    if failure == "stale":
        native.motors[-1].age = 1.
    elif failure == "fault":
        native.motors[-1].fault = 1
    else:
        native.motors[-1].pos = float("nan")
    with pytest.raises(backend.BackendError):
        hardware.command(np.zeros(14), np.zeros(14))
    assert len(native.frames) == before
    assert all(m.command == ("read",) for m in native.motors)
    hardware.brake()


def test_invalid_position_velocity_never_queue_commands(description, monkeypatch, fake_clock):
    hardware, native = make_hardware(description, monkeypatch, fake_clock)
    before = len(native.frames)
    for q, v in [(np.ones(14) * 4, np.zeros(14)), (np.zeros(14), np.ones(14) * 2),
                 (np.zeros(7), np.zeros(7)), (np.full(14, np.nan), np.zeros(14))]:
        with pytest.raises((ValueError, backend.BackendError)):
            hardware.command(q, v)
    assert len(native.frames) == before


def test_gripper_changes_only_selected_side_and_slews(description, monkeypatch, fake_clock):
    hardware, native = make_hardware(description, monkeypatch, fake_clock, ("right",))
    before = len(native.frames)
    hardware.gripper("right", 2.)
    assert len(native.frames) == before
    assert not hardware.grippers_settled()
    fake_clock.sleep(.02)
    hardware.command(np.zeros(7), np.zeros(7))
    command = native.frames[-1][1]["ros_right_8"]
    assert 0. < command[1] <= .025 + 1e-9
    assert hardware.gripper_positions() == {"right": 0.}
    native.motors[-1].pos = 2.
    assert hardware.grippers_settled()
    with pytest.raises(backend.BackendError):
        hardware.gripper("left", .5)
    with pytest.raises(backend.BackendError):
        hardware.gripper("right", 3.)


def test_binding_failure_brakes_only_recognized_selected_motors(description, monkeypatch, fake_clock):
    native = Native(("right",), fake_clock)
    native.motors.append(Motor("unknown_motor", 99, fake_clock))
    with pytest.raises(backend.BackendError):
        make_hardware(description, monkeypatch, fake_clock, ("right",), native)
    assert len(native.frames) == 3
    for _, commands in native.frames:
        assert commands["unknown_motor"] is None
        assert all(commands[f"ros_right_{i}"] == ("brake",) for i in range(1, 9))


def test_stable_serial_mapping_works_with_reversed_acm_order(tmp_path, description):
    config, _ = backend.combined_motor_config(description, ("left", "right"))
    by_id = tmp_path / "by-id";by_id.mkdir()
    devices = [tmp_path / f"ttyACM{i}" for i in range(2)]
    for device in devices:
        device.touch()
    (by_id / "usb_board_12345-if00").symlink_to(devices[0])
    (by_id / "usb_board_12345-if02").symlink_to(devices[1])
    result, rows = resolve_ports(config, {"board_serial": "12345"},
                                 [str(devices[1]), str(devices[0])], by_id=by_id,
                                 access=lambda *_: True)
    ports = result["robot"]["CANboard"]["No_1_CANboard"]["CANport"]
    assert [p["serial_id"] for p in ports.values()] == [2, 1]
    assert [row["device"] for row in rows] == [str(device) for device in devices]
    with pytest.raises(PermissionError):
        resolve_ports(config, {"board_serial": "12345"}, [str(p) for p in devices],
                      by_id=by_id, access=lambda *_: False)
