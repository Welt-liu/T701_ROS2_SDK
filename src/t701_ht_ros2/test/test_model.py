"""Actual geometry, coordinate and asset tests; never import the motor driver."""
from pathlib import Path
import hashlib
import importlib
import importlib.abc
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from t701_ht_ros2.model import ArmModel, SelfCollisionGuard


SOURCE_DESCRIPTION = Path(__file__).resolve().parents[2] / "t701_ht_description"
if SOURCE_DESCRIPTION.is_dir():
    DESCRIPTION = SOURCE_DESCRIPTION
else:
    from ament_index_python.packages import get_package_share_directory
    DESCRIPTION = Path(get_package_share_directory("t701_ht_description"))


@pytest.fixture(scope="module")
def model():
    return ArmModel(DESCRIPTION)


def test_model_import_does_not_load_driver():
    class ForbidDriver(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, *args, **kwargs):
            if fullname == "hightorque_robot":
                raise AssertionError("Motor driver must not be imported")
    finder = ForbidDriver()
    sys.meta_path.insert(0, finder)
    try:
        importlib.reload(sys.modules["t701_ht_ros2.model"])
    finally:
        sys.meta_path.remove(finder)


def test_description_has_all_meshes_and_original_axis_names():
    root = ET.parse(DESCRIPTION / "urdf/T701-HT.urdf").getroot()
    assert root.get("name") == "T701-HT"
    expected = {f"A_{side}_joint{index}_link" for side in ("l", "r") for index in range(1, 8)}
    assert {joint.get("name") for joint in root.findall("joint")} == expected
    assert len(root.findall("link")) == 15
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        assert filename.startswith("package://t701_ht_description/meshes/")
        assert (DESCRIPTION / "meshes" / Path(filename).name).is_file()
    provenance = json.loads((DESCRIPTION / "source_provenance.json").read_text())
    for name, digest in provenance["original_files"].items():
        if name.startswith("Origami V2/meshes/"):
            assert hashlib.sha256((DESCRIPTION / "meshes" / Path(name).name).read_bytes()).hexdigest() == digest


def test_motor_coordinate_zero_vr_and_signed_ros_state(model):
    zero, vr = model.zeros(), model.vr_pose()
    assert zero.shape == vr.shape == (14,)
    np.testing.assert_allclose(zero, 0.)
    assert vr[6] == np.pi / 2. and vr[13] == -np.pi / 2.
    assert model.names[0] == "left_joint1" and model.names[-1] == "right_joint7"
    names, positions, velocities = model.joint_states(vr, np.ones(14))
    assert names[0] == "A_l_joint1_link" and names[-1] == "A_r_joint7_link"
    np.testing.assert_allclose(positions, vr * model.signs)
    np.testing.assert_allclose(velocities, model.signs)
    np.testing.assert_allclose(model.motor_to_model(vr)[model.q_indices], positions)
    model.validate(vr)
    assert model.collision(zero, zero) is None
    assert model.collision(zero, vr) is None
    assert model.collision(vr, zero) is None


def test_forward_and_gravity_have_motor_axes_and_finite_values(model):
    forward = model.forward(model.zeros())
    assert set(forward) == {"left", "right"}
    for matrix in forward.values():
        assert matrix.shape == (4, 4) and np.all(np.isfinite(matrix))
        np.testing.assert_allclose(matrix[3], [0., 0., 0., 1.])
        np.testing.assert_allclose(matrix[:3, :3] @ matrix[:3, :3].T, np.eye(3), atol=1e-10)
    np.testing.assert_allclose(forward["left"][:2, 3], [0.00075, 0.185], atol=1e-7)
    np.testing.assert_allclose(forward["right"][:2, 3], [0.00075, -0.185], atol=1e-7)
    gravity = model.gravity(model.zeros())
    assert gravity.shape == (14,) and np.all(np.isfinite(gravity))
    assert np.all(abs(gravity) <= model.torque)


def test_three_collision_categories_and_only_rigid_neighbors_excluded(model):
    assert len(model.guard.active_pair_indices) == 91
    assert model.guard.margin == .001
    rng = np.random.default_rng(50)
    seen = set()
    for _ in range(400):
        q = rng.uniform(model.lower, model.upper)
        collision = model.guard.first_collision(model.split(q))
        if collision:
            a, b = collision["pair"]
            seen.add("body" if "base_link" in (a, b) else "cross" if a[0] != b[0] else "self")
            assert collision["distance"] <= model.guard.margin
        if len(seen) == 3:
            break
    assert seen == {"body", "cross", "self"}


def test_safe_endpoints_do_not_hide_collision_between_them(model):
    start = np.array([-0.9573252301535251, -1.2760396688679512, -1.342917829452853,
                      -2.069940562364428, -0.31000712050358525, -0.0565013687546031,
                      2.350217826726975, 2.5029664993563427, -0.10081485882600408,
                      0.590377278947978, 2.1416460573107727, 1.1405664936795261,
                      -0.11405862023678603, -2.3260869796636046])
    target = np.array([-2.9394551167330523, -0.5842151251568661, -1.4287956194493419,
                       0.4634427319122456, -0.12552644723994266, 0.2790867773832627,
                       2.1741398944845733, -0.7148706714678922, 0.1455849645763358,
                       -1.4868677234182253, 1.183388377425912, 0.11497667573682135,
                       -0.4908888149545969, -3.005470114459219])
    assert model.guard.first_collision(model.split(start)) is None
    assert model.guard.first_collision(model.split(target)) is None
    collision = model.collision(start, target)
    assert collision is not None and 0. < collision["sample"] < 1.


def test_single_selected_arm_has_no_invented_other_state():
    for side in ("left", "right"):
        model = ArmModel(DESCRIPTION, sides=(side,))
        assert model.count == 7 and set(model.split(model.zeros())) == {side}
        assert model.names[0] == f"{side}_joint1"
        assert len(model.guard.active_pair_indices) == 21
        assert model.collision(model.zeros(), model.vr_pose()) is None
        assert set(model.forward(model.zeros())) == {side}


def test_invalid_arrays_and_outside_joint_limits_rejected(model):
    for q in ([0.] * 13, [float("nan")] * 14, [float("inf")] * 14):
        with pytest.raises(ValueError):
            model.validate(q)
    q = model.zeros()
    q[0] = model.lower[0] - .01
    with pytest.raises(ValueError):
        model.collision(model.zeros(), q)
    with pytest.raises(ValueError):
        ArmModel(DESCRIPTION, sides=("left", "left"))
    with pytest.raises(ValueError):
        model.guard.validate_path(model.split(model.zeros()), model.split(model.vr_pose()), max_step=.1)


def test_model_config_paths_are_independent_and_self_contained(model):
    for side, config in model.configs.items():
        path = model.config_paths[side]
        assert (path.parent / config["urdf"]["file_path"]).resolve().is_file()
        assert (path.parent / config["robot"]["param_file"]).resolve().is_file()
        assert str(DESCRIPTION.resolve()) in str((path.parent / config["urdf"]["file_path"]).resolve())
