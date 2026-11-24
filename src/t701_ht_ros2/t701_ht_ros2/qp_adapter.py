"""Kinematic adapter for the unchanged SDK 2.0 velocity-level OSQP solver.

The mathematical solver in qp_ik.py is copied byte for byte from the Python
SDK. This adapter supplies its existing seven-axis T701-HT-shaped interface
without importing a motor SDK. Joint vectors always use motor coordinates.
"""
from __future__ import annotations

import numpy as np


# Original SDK 2.0 scripts/Lib/qp_ik.py, included in this independent package.
SDK_QP_SHA256 = "7c7d8d91fb9c79d7f1a71dea5129628d7c8ae22e790e6b31466c6a5a84fd31a9"


def _vector(value, count, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (count,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} needs {count} finite values")
    return result.copy()


class ArmQPAdapter:
    """Expose one selected arm while collision checks retain all known arms.

    The controller refreshes set_context with the complete measured motor vector
    before each QP step. The other selected arm remains at its measured position
    for local checks; the engine also validates the combined command before
    sending either side. In single-arm mode, unseen-arm collision is unknown.
    """

    motor_count = 7

    def __init__(self, model, side):
        if side not in model.sides:
            raise ValueError("QP side must be selected in the model")
        self.arm_model = model
        self.side = side
        self.index = model.sides.index(side) * 7
        self.indices = slice(self.index, self.index + 7)
        self.velocity_limits = model.velocity[self.indices].copy()
        self.acceleration_limits = model.acceleration[self.indices].copy()
        self.joint_limits = {"lower": model.lower[self.indices].copy(),
                             "upper": model.upper[self.indices].copy()}
        self.full_q = model.zeros()
        self.data = model.model.createData()
        self.frame = model.end_frames[side]
        self.collision_blocked = False
        self.collision_reason = None

    def set_context(self, full_q):
        full_q = _vector(full_q, self.arm_model.count, "Measured joint vector")
        self.arm_model.validate(full_q)
        self.full_q = full_q
        self.collision_blocked = False
        self.collision_reason = None

    def _full(self, joint):
        full = self.full_q.copy()
        full[self.indices] = _vector(joint, 7, "QP arm joint vector")
        return full

    def forward_kinematics(self, joint):
        model = self.arm_model
        q = model.motor_to_model(self._full(joint))
        model.pin.forwardKinematics(model.model, self.data, q)
        model.pin.updateFramePlacements(model.model, self.data)
        transform = np.array(self.data.oMf[self.frame].homogeneous, copy=True)
        return {"transform": transform, "position": transform[:3, 3].copy(),
                "rotation": transform[:3, :3].copy()}

    def get_jacobian(self, joint):
        model = self.arm_model
        q = model.motor_to_model(self._full(joint))
        full = model.pin.computeFrameJacobian(
            model.model, self.data, q, self.frame, model.pin.LOCAL_WORLD_ALIGNED,
        )
        columns = model.v_indices[self.indices]
        signs = model.signs[self.indices]
        return np.asarray(full[:, columns], dtype=float) * signs[None, :]

    def check_self_collision(self, next_joint, velocity=None):
        model = self.arm_model
        target = self._full(next_joint)
        reason = model.collision(self.full_q, target)
        if reason is None and velocity is not None:
            future = target.copy()
            future[self.indices] += .10 * _vector(velocity, 7, "QP velocity")
            reason = model.collision(target, future)
        self.collision_blocked = reason is not None
        self.collision_reason = reason
        return reason
