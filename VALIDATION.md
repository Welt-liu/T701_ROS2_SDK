# Validation record / 验证记录

Date / 日期: 2026-11-21

## Local build and simulation / 本机构建与模拟

- Ubuntu 22.04, ROS 2 Humble, Python 3.10.
- Both `openarm_description` and `openarm_ros2` build with colcon.
- 104 automated tests pass: geometry, motor coordinate signs, exact SDK QP
  provenance, solver convergence, trajectories, mock hardware, ROS message
  validation, cancellation, home/brake, and stop during planning.
- Two-process DDS test passes: 14 motor states, FollowJointTrajectory
  completion, home service, and confirmed damping-brake mode.
- Two-process QP demo passes: measured tool-pose arrival, subsequent home,
  gripper completion, and damping brake.
- RViz2 displays the complete robot model with Global Status OK. Software
  rendering (`OPENARM_SOFTWARE_RENDERING=1`) was used for this display.
- Simulation imports no `hightorque_robot` extension and opens no CAN port.

## Hardware scope / 实机范围

Native SDK calls are validated with fake motors, including stale feedback,
selected-arm isolation and explicit damping brake. This record does not claim
physical motor movement, collision contact, braking-distance or hardware
tracking tests. Hardware operation requires a separate supervised commissioning
run with the selected board and motors.

## Provenance / 来源

The QP source SHA-256 is
`172a42b70668cfc82ea0411fd829f3a8953ed747dd1a7b011a09c155e78b9e07`.
It matches Python SDK 2.0 `scripts/Lib/qp_ik.py` byte for byte.
Model and wheel source hashes are stored in the supplied provenance manifests.

## ThinkPad installation and simulation / ThinkPad 安装与模拟

- ROS 2 Humble desktop, RViz2, Pinocchio 4.1.0 and FCL installed.
- Workspace-local Python 3.10 environment configured; both ROS packages build.
- All 104 automated tests pass on the ThinkPad (47.10 seconds).
- Separate-process simulated state, joint motion, QP IK, gripper, home and damping-brake checks pass.
- RViz2 launches on the desktop with OpenGL 4.5 and exits normally.
- All distribution source/document/image files match the delivery manifest.
- No physical motor commands were sent during these checks.
- Chinese and English user guides contain matching command blocks and five local images each.

## Explicit modes and layout / 明确模式与目录分类

- `start.sh sim` and `start.sh real` select independent backends and default DDS domains 20 and 21.
- Mode-specific demos confirm the driver diagnostic backend before sending a command. A mismatch is rejected.
- All tests are under `src/openarm_ros2/test/`; 104 tests pass locally.
- Model/configuration resources and runtime control remain separate ROS packages. Duplicate resources and unused encoder configurations are omitted.
- Clean build of both packages succeeds. QP source remains byte-identical to Python SDK 2.0.
- Homing holds the measured starting posture while planning, including transition from QP completion.
- Cross-process mode-specific simulated state, joint motion, QP IK, home and damping-brake checks pass.
- An explicitly real-targeted client on a simulation domain is rejected before commands are sent.
- This change does not include physical motor movement verification.
