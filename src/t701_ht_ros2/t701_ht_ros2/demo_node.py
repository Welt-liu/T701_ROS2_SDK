"""Small ROS 2 client demonstrations; the default mode only reads state."""

import argparse
import copy
import math
import signal
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation
import numpy as np


def point_at(seconds, positions):
    point = JointTrajectoryPoint()
    point.positions = [float(q) for q in positions]
    point.velocities = [0.] * len(positions)
    nanoseconds = round(seconds * 1e9)
    point.time_from_start.sec, point.time_from_start.nanosec = divmod(nanoseconds, 1_000_000_000)
    return point


def interrupt_client(_signum, _frame):
    raise KeyboardInterrupt


class DemoNode(Node):
    def __init__(self):
        super().__init__('t701_ht_demo')
        self.latest = None
        self.tool_poses = {}
        self.last_diagnostics = None
        self.diagnostics_time = 0.
        self.subscription = self.create_subscription(
            JointState, '/t701_ht/motor_states', self._state, 10)
        self.action = ActionClient(self, FollowJointTrajectory,
                                   '/t701_ht/follow_joint_trajectory')
        self.pose_subscriptions = [self.create_subscription(
            PoseStamped, f'/t701_ht/{side}_tool_pose',
            lambda message, side=side: self._pose(side, message), 10)
            for side in ('left', 'right')]
        self.diagnostic_subscription = self.create_subscription(
            DiagnosticArray, '/diagnostics', self._diagnostics, 10)

    def _state(self, message):
        self.latest = message

    def _pose(self, side, message):
        self.tool_poses[side] = message

    def _diagnostics(self, message):
        for state in message.status:
            if state.name == 't701_ht_driver':
                self.last_diagnostics = {value.key: value.value for value in state.values}
                self.diagnostics_time = time.monotonic()

    def wait_state(self, seconds=5.):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and self.latest is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
        if self.latest is None:
            raise RuntimeError('No /t701_ht/motor_states received; start sdk.launch.py first.')
        return self.latest

    def require_backend(self, expected, seconds=15.):
        """Confirm the driver's mode before any example can send a command."""
        if expected == 'auto':
            return
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            status = self.last_diagnostics or {}
            actual = status.get('backend')
            if actual is not None:
                if actual != expected:
                    raise RuntimeError(f'Example target is {expected}, but driver is {actual}; no command sent.')
                self.get_logger().info(f'Confirmed {actual} driver.')
                return
        raise RuntimeError(f'No confirmed {expected} driver; start it in the matching sim/real entry.')

    def call_trigger(self, name):
        client = self.create_client(Trigger, name)
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError(f'Service unavailable: {name}')
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
            if not future.done() or future.result() is None:
                raise RuntimeError(f'Service did not respond: {name}')
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(response.message)
        finally:
            self.destroy_client(client)

    def enable(self):
        client = self.create_client(SetBool, '/t701_ht/set_enabled')
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError('Enable service unavailable.')
            request = SetBool.Request()
            request.data = True
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.)
            if not future.done() or future.result() is None:
                raise RuntimeError('Enable service did not respond.')
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            self.get_logger().info(response.message)
        finally:
            self.destroy_client(client)

    def watch(self, duration):
        self.wait_state()
        deadline = time.monotonic() + duration
        report_at = 0.
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
            if time.monotonic() >= report_at:
                values = ', '.join(f'{n}={q:+.3f}' for n, q in
                                   zip(self.latest.name, self.latest.position))
                self.get_logger().info(values)
                report_at = time.monotonic() + 1.

    def wait_home(self, timeout=45.):
        self.latest = None
        requested_at = time.monotonic()
        self.call_trigger('/t701_ht/home')
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < timeout:
            rclpy.spin_once(self, timeout_sec=.1)
            message = self.latest
            status = self.last_diagnostics or {}
            fresh = self.diagnostics_time > requested_at
            if fresh and (status.get('last_error') or status.get('mode') == 'fault'):
                raise RuntimeError(status.get('last_error') or 'Driver fault during return')
            if message and message.position and all(abs(q) <= .03 for q in message.position):
                if (message.velocity and all(abs(v) <= .08 for v in message.velocity)
                        and fresh and status.get('mode') == 'braked'):
                    self.get_logger().info('Selected arms and grippers are at zero; damping brake confirmed by the driver.')
                    return
        raise RuntimeError('No confirmed zero arrival; check /diagnostics.')

    def motion(self, mode, duration, amplitude):
        state = self.wait_state()
        names = list(state.name)
        origin = list(state.position)
        if not names or len(names) != len(origin):
            raise RuntimeError('Invalid motor state.')
        moving = [i for i, name in enumerate(names) if name.endswith('_joint1')]
        if not moving:
            raise RuntimeError('No joint1 in motor_states.')
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names
        goal.trajectory.points = [point_at(0., origin)]
        fractions = (.5, 1.) if mode == 'joint' else (.25, .5, .75, 1.)
        for fraction in fractions:
            q = origin.copy()
            offset = (amplitude if fraction == .5 else 0.) if mode == 'joint' else (
                amplitude * math.sin(2. * math.pi * fraction))
            for index in moving:
                q[index] += -offset if names[index].startswith('right_') else offset
            goal.trajectory.points.append(point_at(fraction * duration, q))
        if not self.action.wait_for_server(timeout_sec=5.):
            raise RuntimeError('Trajectory action unavailable.')
        future = self.action.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.)
        if not future.done() or future.result() is None or not future.result().accepted:
            raise RuntimeError('Trajectory was rejected; check enabled state and diagnostics.')
        handle = future.result()
        result_future = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration + 20.)
            if not result_future.done() or result_future.result() is None:
                cancel = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel, timeout_sec=3.)
                raise RuntimeError('Trajectory timeout; cancellation and smooth return requested.')
            result = result_future.result().result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(result.error_string or f'Trajectory error {result.error_code}')
            self.get_logger().info('Trajectory completed; returning selected arms to zero.')
            self.wait_home()
        except KeyboardInterrupt:
            if rclpy.ok():
                future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future, timeout_sec=3.)
                self.get_logger().info('Cancellation sent; the driver owns the smooth return and brake.')
            raise

    def gripper(self, side, target):
        self.wait_state()
        if not any(name.startswith(side + '_') for name in self.latest.name):
            raise ValueError(f'{side} arm is not selected by the driver')
        publisher = self.create_publisher(Float64, f'/t701_ht/{side}_gripper', 10)
        try:
            deadline = time.monotonic() + 3.
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
            if publisher.get_subscription_count() == 0:
                raise RuntimeError('Gripper subscription unavailable.')
            message = Float64()
            message.data = target
            publisher.publish(message)
            self.get_logger().info('Gripper target sent; this one-shot topic has no arrival acknowledgement.')
            rclpy.spin_once(self, timeout_sec=.2)
        finally:
            self.destroy_publisher(publisher)

    def ik(self, side, duration, distance=.01):
        """A small base-Z pose step, solved by the driver's SDK 2.0 QP, then home."""
        state = self.wait_state()
        if not any(name.startswith(side + '_') for name in state.name):
            raise ValueError(f'{side} arm is not selected by the driver')
        deadline = time.monotonic() + 5.
        while side not in self.tool_poses and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=.1)
        if side not in self.tool_poses:
            raise RuntimeError(f'No /t701_ht/{side}_tool_pose received')
        target = copy.deepcopy(self.tool_poses[side])
        if target.header.frame_id != 'base_link':
            raise ValueError('tool_pose must be expressed in base_link')
        # From the straight-down hardware zero, a pure upward displacement at
        # fixed orientation can be singular/unreachable. Use a tiny FK-derived
        # J1 displacement as a guaranteed nearby pose target for the QP demo.
        from ament_index_python.packages import get_package_share_directory
        from .model import ArmModel
        import numpy as np
        from scipy.spatial.transform import Rotation
        sides = tuple(side_name for side_name in ('left', 'right')
                      if any(name.startswith(side_name + '_') for name in state.name))
        model = ArmModel(get_package_share_directory('t701_ht_description'), sides)
        measured = dict(zip(state.name, state.position))
        q = np.array([measured[name] for name in model.names])
        index = model.sides.index(side) * 7
        q[index] += .06 if side == 'left' else -.06
        transform = model.forward(q)[side]
        target.pose.position.x, target.pose.position.y, target.pose.position.z = transform[:3, 3]
        quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
        target.pose.orientation.x, target.pose.orientation.y, target.pose.orientation.z, target.pose.orientation.w = quat
        publisher = self.create_publisher(PoseStamped, f'/t701_ht/{side}_target_pose', 10)
        try:
            deadline = time.monotonic() + 3.
            while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
            if publisher.get_subscription_count() == 0:
                raise RuntimeError('QP IK target subscription unavailable')
            sent_at = time.monotonic()
            publisher.publish(target)
            self.get_logger().info('QP IK requested: nearby FK-derived tool pose; monitoring measured arrival.')
            deadline = sent_at + max(25., duration)
            reached = False
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=.1)
                status = self.last_diagnostics
                if not status or self.diagnostics_time <= sent_at:
                    continue
                mode = status.get('mode')
                if status.get('last_error') or mode in ('fault', 'collision_hold', 'ik_hold'):
                    raise RuntimeError(status.get('last_error') or f'QP IK stopped: {mode}')
                current = self.tool_poses.get(side)
                if current is not None and mode == 'idle' and pose_distance(current, target)[0] <= .002:
                    if pose_distance(current, target)[1] <= .03:
                        reached = True
                        break
                if mode in ('disabled', 'braked'):
                    raise RuntimeError('Driver disabled or braked before QP target arrival')
            if not reached:
                # A healthy driver owns the validated return path. Fault states are
                # never re-enabled or blindly sent toward zero by this client.
                if self.last_diagnostics and self.last_diagnostics.get('enabled') == 'True':
                    self.call_trigger('/t701_ht/home')
                raise RuntimeError('QP IK target did not arrive before timeout')
            self.get_logger().info('QP IK target reached; returning selected arms to zero.')
            self.wait_home()
        except KeyboardInterrupt:
            if rclpy.ok():
                status = self.last_diagnostics or {}
                if status.get('mode') != 'fault' and status.get('enabled') == 'True':
                    self.call_trigger('/t701_ht/home')
            raise
        finally:
            self.destroy_publisher(publisher)


def pose_distance(first, second):
    a, b = first.pose, second.pose
    position = np.linalg.norm(np.array([a.position.x, a.position.y, a.position.z]) -
                              np.array([b.position.x, b.position.y, b.position.z]))
    qa = [a.orientation.x, a.orientation.y, a.orientation.z, a.orientation.w]
    qb = [b.orientation.x, b.orientation.y, b.orientation.z, b.orientation.w]
    rotation = (Rotation.from_quat(qa).inv() * Rotation.from_quat(qb)).magnitude()
    return float(position), float(rotation)


def parser():
    result = argparse.ArgumentParser(description='T701-HT ROS 2 examples; state-only by default.',
                                     allow_abbrev=False)
    result.add_argument('--mode', choices=('state', 'enable', 'home', 'stop', 'joint',
                                          'trajectory', 'gripper', 'ik'), default='state')
    result.add_argument('--target', choices=('auto', 'sim', 'real'), default='auto',
                        help='required driver mode; sim/real entry sets this automatically')
    result.add_argument('--duration', type=float, default=5.)
    result.add_argument('--amplitude', type=float, default=.10, help='J1 amplitude in radians')
    result.add_argument('--side', choices=('left', 'right'), default='left')
    result.add_argument('--gripper', type=float, default=.2, help='gripper motor radians')
    return result


def main(args=None):
    import sys
    argv = sys.argv if args is None else ['t701_ht_demo', *args]
    options = parser().parse_args(remove_ros_args(argv)[1:])
    if not math.isfinite(options.duration) or options.duration <= 0.:
        raise ValueError('duration must be finite and positive')
    if not math.isfinite(options.amplitude) or not 0. < options.amplitude <= .2:
        raise ValueError('demo amplitude must be between 0 and 0.2 rad')
    if not math.isfinite(options.gripper):
        raise ValueError('gripper target must be finite')
    # Keep the client context alive long enough to deliver a Ctrl+C cancellation.
    # The default rclpy SIGINT hook would close DDS before cancel_goal_async.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    previous_term = signal.signal(signal.SIGTERM, interrupt_client)
    node = DemoNode()
    try:
        node.require_backend(options.target)
        if options.mode == 'state':
            node.watch(options.duration)
        elif options.mode == 'enable':
            node.enable()
        elif options.mode == 'home':
            node.wait_home()
        elif options.mode == 'stop':
            node.call_trigger('/t701_ht/stop')
        elif options.mode == 'gripper':
            node.gripper(options.side, options.gripper)
            node.watch(options.duration)
            node.wait_home()
        elif options.mode == 'ik':
            node.ik(options.side, options.duration)
        else:
            node.motion(options.mode, options.duration, options.amplitude)
    except KeyboardInterrupt:
        if options.mode == 'gripper' and rclpy.ok():
            state = node.last_diagnostics or {}
            if state.get('mode') not in ('fault', 'braked') and state.get('enabled') == 'True':
                node.call_trigger('/t701_ht/home')
        pass
    except Exception as error:
        node.get_logger().error(str(error))
        raise SystemExit(1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_term)


if __name__ == '__main__':
    main()
