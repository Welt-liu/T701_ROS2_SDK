"""Stable CAN-to-USB mapping adapted from T701-HT SDK 2.0.

Resolution never opens a device. Native SDK indices are assigned only in a
temporary YAML file; source configuration and OS serial permissions are untouched.
"""
from __future__ import annotations

from contextlib import contextmanager
import copy
import os
from pathlib import Path
import tempfile

import yaml


def read_yaml(path):
    with Path(path).open(encoding="utf-8") as stream:
        result = yaml.safe_load(stream)
    if not isinstance(result, dict) or not isinstance(result.get("robot"), dict):
        raise ValueError(f"Invalid robot YAML: {path}")
    return result


def native_order(prefix, special=False, *, scandir=os.scandir,
                 sys_root=Path("/sys/class/tty")):
    """Match SDK readdir/reverse order and supported USB VID/PID filtering."""
    prefix = Path(prefix)
    with scandir(prefix.parent) as entries:
        names = [entry.name for entry in entries if entry.name.startswith(prefix.name)]
    accepted_vids = {0xfae1} if special else {0xcaf1, 0xcae1}
    result = []
    for name in reversed(names):
        node = (sys_root / name / "device").resolve()
        for ancestor in (node, *node.parents):
            vid_path, pid_path = ancestor / "idVendor", ancestor / "idProduct"
            if vid_path.is_file() and pid_path.is_file():
                vid = int(vid_path.read_text().strip(), 16)
                pid = int(pid_path.read_text().strip(), 16)
                if vid in accepted_vids and pid == 0xffff:
                    result.append(str(prefix.parent / name))
                break
    return result


def resolve_ports(config, bindings, order, *, by_id=Path("/dev/serial/by-id"),
                  access=os.access):
    """Map logical CAN1..7 through board identity, not changing ACM ordering."""
    result = copy.deepcopy(config)
    serial = bindings.get("board_serial")
    if not isinstance(serial, str) or not serial or any(c in serial for c in "*?[]/"):
        raise ValueError("serial_binding.yaml must contain a specific board_serial")
    seen, rows = set(), []
    for board in result["robot"]["CANboard"].values():
        for label, port in board["CANport"].items():
            can = port.get("can_port")
            if type(can) is not int or not 1 <= can <= 7:
                raise ValueError(f"{label}: can_port must be CAN1..7")
            pattern = f"*_{serial}-if{2 * (can - 1):02x}"
            links = list(by_id.glob(pattern))
            if len(links) != 1:
                raise RuntimeError(f"CAN{can}: stable USB interface missing or ambiguous ({pattern})")
            device = str(links[0].resolve(strict=True))
            if device not in order:
                raise RuntimeError(f"CAN{can}: {device} is absent from SDK serial list")
            if device in seen:
                raise ValueError(f"A serial device is assigned twice: {device}")
            if not access(device, os.R_OK | os.W_OK):
                raise PermissionError(f"{device} is not readable/writable; activate the dialout group")
            seen.add(device)
            port["serial_id"] = order.index(device) + 1
            rows.append({"can_port": can, "device": device,
                         "serial_id": port["serial_id"], "stable_path": str(links[0])})
    if not rows:
        raise ValueError("No CAN channels configured")
    return result, rows


@contextmanager
def prepared_config(config, bindings_path):
    """Yield resolved temporary configuration; never import or create a Robot."""
    with Path(bindings_path).open(encoding="utf-8") as stream:
        bindings = yaml.safe_load(stream)
    if not isinstance(bindings, dict):
        raise ValueError("Invalid serial binding YAML")
    prefix = config["robot"]["Serial_Type"]
    special = config["robot"].get("board_special_flag", False)
    order = native_order(prefix, special)
    resolved, rows = resolve_ports(config, bindings, order)
    if native_order(prefix, special) != order:
        raise RuntimeError("Serial interfaces changed during resolution; retry when stable")
    with tempfile.TemporaryDirectory(prefix="t701_ht_ros2_serial_") as directory:
        path = Path(directory) / "motor_runtime.yaml"
        path.write_text(yaml.safe_dump(resolved, allow_unicode=True, sort_keys=False), encoding="utf-8")
        yield path, rows
