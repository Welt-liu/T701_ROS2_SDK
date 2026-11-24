"""Original SDK OSQP, motor coordinates and collision integration; no CAN."""
from pathlib import Path
import hashlib
import importlib
import importlib.abc
import sys

import numpy as np
import pytest

from t701_ht_ros2.model import ArmModel
from t701_ht_ros2.qp_adapter import ArmQPAdapter, SDK_QP_SHA256
from t701_ht_ros2.qp_ik import PinocchioOSQPIK


DESCRIPTION = Path(__file__).resolve().parents[2] / "t701_ht_description"


@pytest.fixture(scope="module")
def model():
    return ArmModel(DESCRIPTION)


def test_qp_solver_is_byte_identical_to_sdk_2_0():
    from t701_ht_ros2 import qp_ik
    assert hashlib.sha256(Path(qp_ik.__file__).read_bytes()).hexdigest() == SDK_QP_SHA256


def test_qp_import_never_imports_native_robot():
    class ForbidNative(importlib.abc.MetaPathFinder):
        def find_spec(self, fullname, *args, **kwargs):
            if fullname == "hightorque_robot":
                raise AssertionError("QP must not import motor SDK")
    finder = ForbidNative()
    sys.meta_path.insert(0, finder)
    try:
        importlib.reload(sys.modules["t701_ht_ros2.qp_adapter"])
        importlib.reload(sys.modules["t701_ht_ros2.qp_ik"])
    finally:
        sys.meta_path.remove(finder)


@pytest.mark.parametrize("side", ["left", "right"])
def test_adapter_fk_matches_full_model_and_jacobian_finite_differences(model, side):
    q = model.vr_pose()
    adapter = ArmQPAdapter(model, side)
    adapter.set_context(q)
    offset = model.sides.index(side) * 7
    local = q[offset:offset + 7]
    fk = adapter.forward_kinematics(local)["transform"]
    np.testing.assert_allclose(fk, model.forward(q)[side], atol=1e-12)
    jacobian = adapter.get_jacobian(local)
    assert jacobian.shape == (6, 7)
    epsilon = 1e-6
    for axis in range(7):
        shifted = local.copy();shifted[axis] += epsilon
        next_fk = adapter.forward_kinematics(shifted)["transform"]
        numerical = (next_fk[:3, 3] - fk[:3, 3]) / epsilon
        np.testing.assert_allclose(jacobian[:3, axis], numerical, atol=1e-6)


@pytest.mark.parametrize("side", ["left", "right"])
def test_same_qp_converges_with_velocity_acceleration_and_collision_checks(model, side):
    q = model.vr_pose()
    offset = model.sides.index(side) * 7
    adapter = ArmQPAdapter(model, side)
    adapter.set_context(q)
    target = adapter.forward_kinematics(q[offset:offset + 7])["transform"].copy()
    target[2, 3] += .01
    solver = PinocchioOSQPIK(
        adapter, max_joint_velocity=.5, max_joint_acceleration=2.,
        max_linear_velocity=.10, max_angular_velocity=.5,
        singularity_threshold=.02, singularity_damping=.01,
    )
    solver.reset(posture_reference=q[offset:offset + 7])
    previous_velocity = np.zeros(7)
    other = q.copy()
    for _ in range(150):
        adapter.set_context(q)
        start = q.copy()
        result = solver.step(q[offset:offset + 7], target, .02)
        assert max(abs(result.joint_velocity)) <= .5 + 1e-8
        assert max(abs(result.joint_velocity - previous_velocity)) <= .04 + 1e-7
        q[offset:offset + 7] = result.joint_position
        model.validate(q)
        assert model.collision(start, q) is None
        # Solving one side must preserve the measured other arm.
        if offset == 0:
            np.testing.assert_array_equal(q[7:], other[7:])
        else:
            np.testing.assert_array_equal(q[:7], other[:7])
        previous_velocity = result.joint_velocity.copy()
        if result.position_error < .002 and result.rotation_error < .03 and max(abs(result.joint_velocity)) < .03:
            break
    else:
        pytest.fail("Original SDK QP failed to reach a nearby reachable target")


def test_adapter_collision_rejects_and_resets_qp_velocity(model, monkeypatch):
    q = model.vr_pose()
    adapter = ArmQPAdapter(model, "left")
    adapter.set_context(q)
    solver = PinocchioOSQPIK(adapter)
    target = adapter.forward_kinematics(q[:7])["transform"].copy()
    target[2, 3] += .01
    monkeypatch.setattr(model, "collision", lambda start, target: {"pair": ("arm", "body")})
    with pytest.raises(RuntimeError, match="QP IK"):
        solver.step(q[:7], target, .02)
    assert adapter.collision_blocked
    np.testing.assert_array_equal(solver.previous_velocity, np.zeros(7))


def test_adapter_predicts_velocity_and_retains_full_arm_context(model, monkeypatch):
    q = model.vr_pose()
    adapter = ArmQPAdapter(model, "left")
    adapter.set_context(q)
    calls = []
    def check(start, target):
        calls.append((start.copy(), target.copy()))
        return None if len(calls) == 1 else {"pair": ("l_link", "r_link")}
    monkeypatch.setattr(model, "collision", check)
    target = q[:7].copy();target[0] += .001
    velocity = np.r_[.1, np.zeros(6)]
    assert adapter.check_self_collision(target, velocity)["pair"] == ("l_link", "r_link")
    assert len(calls) == 2
    np.testing.assert_array_equal(calls[0][0], q)
    for start, end in calls:
        np.testing.assert_array_equal(start[7:], q[7:])
        np.testing.assert_array_equal(end[7:], q[7:])
    np.testing.assert_allclose(calls[1][1][0], target[0] + .01)


def test_invalid_adapter_side_context_and_shapes_fail(model):
    with pytest.raises(ValueError):ArmQPAdapter(model, "unknown")
    adapter = ArmQPAdapter(model, "left")
    with pytest.raises(ValueError):adapter.set_context(np.zeros(7))
    with pytest.raises(ValueError):adapter.forward_kinematics(np.zeros(6))
    with pytest.raises(ValueError):adapter.get_jacobian(np.full(7, np.nan))
