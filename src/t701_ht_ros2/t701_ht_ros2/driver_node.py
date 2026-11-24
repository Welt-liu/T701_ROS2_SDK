"""ROS 2 boundary for the standalone SDK; hardware loading is always explicit."""

from pathlib import Path
import math
import signal
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import FollowJointTrajectory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.spatial.transform import Rotation

from .model import ArmModel
from .backend import SimBackend, SDKBackend
from .engine import Engine


def trajectory_points(request, expected_names):
    """Reject unsupported action fields instead of silently ignoring them."""
    trajectory = request.trajectory
    names = list(trajectory.joint_names)
    if len(names) != len(expected_names) or set(names) != set(expected_names):
        raise ValueError('joint_names must contain every selected motor joint exactly once')
    if len(set(names)) != len(names):
        raise ValueError('duplicate joint_names')
    if trajectory.header.stamp.sec or trajectory.header.stamp.nanosec:
        raise ValueError('scheduled header stamps are unsupported; use stamp zero')
    multi = request.multi_dof_trajectory
    if multi.joint_names or multi.points:
        raise ValueError('multi-DOF trajectories are unsupported')
    for field in ('path_tolerance', 'goal_tolerance', 'component_path_tolerance',
                  'component_goal_tolerance'):
        if getattr(request, field, []):
            raise ValueError(f'{field} is unsupported; the SDK configured tolerances apply')
    if request.goal_time_tolerance.sec or request.goal_time_tolerance.nanosec:
        raise ValueError('custom goal_time_tolerance is unsupported')
    if not trajectory.points:
        raise ValueError('trajectory has no points')
    points = []
    previous = -1.
    for point in trajectory.points:
        stamp = point.time_from_start
        t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            raise ValueError('invalid time_from_start')
        if not math.isfinite(t) or t < 0. or t <= previous:
            raise ValueError('time_from_start must be finite, nonnegative and strictly increasing')
        q = list(point.positions)
        v = list(point.velocities)
        if len(q) != len(names) or not np.all(np.isfinite(q)):
            raise ValueError('each point needs one finite position per motor joint')
        if v and (len(v) != len(names) or not np.all(np.isfinite(v))):
            raise ValueError('velocity fields must be absent or complete and finite')
        if point.accelerations or point.effort:
            raise ValueError('explicit accelerations and effort are unsupported')
        item = {'time_from_start': t, 'positions': q}
        if v:
            item['velocities'] = v
        points.append(item)
        previous = t
    if previous <= 0.:
        raise ValueError('trajectory must have a positive duration')
    return names, points


def pose_transform(message):
    """Targets must already be expressed in the fixed robot base frame."""
    if message.header.frame_id != 'base_link':
        raise ValueError('target_pose frame_id must be base_link; transform the pose before publishing')
    p, q = message.pose.position, message.pose.orientation
    position = np.array([p.x, p.y, p.z], dtype=float)
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=float)
    if not np.all(np.isfinite(np.r_[position, quaternion])):
        raise ValueError('target_pose needs a finite position and quaternion')
    norm = float(np.linalg.norm(quaternion))
    if abs(norm - 1.) > 1e-3:
        raise ValueError('target_pose quaternion must be normalized')
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(quaternion / norm).as_matrix()
    transform[:3, 3] = position
    return transform


class DriverNode(Node):
    def __init__(self):
        super().__init__('t701_ht_driver')
        self.declare_parameter('simulate', True)
        self.declare_parameter('arms', 'dual')
        self.declare_parameter('enable_on_start', False)
        self.declare_parameter('state_rate', 50.)
        self.declare_parameter('command_duration', 2.)
        self.declare_parameter('description_dir', '')
        self._lock = threading.RLock()
        self._closing = False
        self._goal_pending = False
        self._active_goal = None
        self._last_error_log = 0.
        self._group = ReentrantCallbackGroup()
        simulate = bool(self.get_parameter('simulate').value)
        self.backend_mode = 'sim' if simulate else 'real'
        selection = str(self.get_parameter('arms').value)
        if selection not in ('left', 'right', 'dual'):
            raise ValueError('arms must be left, right or dual')
        rate = float(self.get_parameter('state_rate').value)
        if not math.isfinite(rate) or not 5. <= rate <= 100.:
            raise ValueError('state_rate must be between 5 and 100 Hz')
        self.command_duration = float(self.get_parameter('command_duration').value)
        if not math.isfinite(self.command_duration) or self.command_duration <= 0.:
            raise ValueError('command_duration must be positive')
        directory = self.get_parameter('description_dir').value
        directory = Path(directory or get_package_share_directory('t701_ht_description'))
        sides = ('left', 'right') if selection == 'dual' else (selection,)
        self.model = ArmModel(directory, sides)
        backend = SimBackend(self.model) if simulate else SDKBackend(self.model, directory)
        try:
            self.engine = Engine(self.model, backend, sample_period=1. / rate)
        except BaseException:
            backend.close()
            raise
        try:
            # Simulation needs no user permission to move; hardware always starts disabled
            # unless enable_on_start:=true was explicitly passed by the operator.
            self.engine.set_enabled(simulate or bool(self.get_parameter('enable_on_start').value))
            self._joint_publisher = self.create_publisher(JointState, '/joint_states', 10)
            self._motor_publisher = self.create_publisher(JointState, '/t701_ht/motor_states', 10)
            self._diagnostics_publisher = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
            self._pose_publishers = {
                side: self.create_publisher(PoseStamped, f'/t701_ht/{side}_tool_pose', 10)
                for side in sides
            }
            self._pose_subscriptions = [self.create_subscription(
                PoseStamped, f'/t701_ht/{side}_target_pose',
                lambda message, side=side: self._target_pose(side, message), 10,
                callback_group=self._group) for side in sides]
            self._command_subscription = self.create_subscription(
                JointState, '/t701_ht/joint_command', self._joint_command, 10,
                callback_group=self._group)
            self._gripper_subscriptions = [self.create_subscription(
                Float64, f'/t701_ht/{side}_gripper',
                lambda msg, side=side: self._gripper_command(side, msg), 10,
                callback_group=self._group) for side in sides]
            self._home_service = self.create_service(
                Trigger, '/t701_ht/home', self._home, callback_group=self._group)
            self._stop_service = self.create_service(
                Trigger, '/t701_ht/stop', self._stop, callback_group=self._group)
            self._enable_service = self.create_service(
                SetBool, '/t701_ht/set_enabled', self._set_enabled, callback_group=self._group)
            self._action = ActionServer(
                self, FollowJointTrajectory, '/t701_ht/follow_joint_trajectory',
                execute_callback=self._execute_goal, goal_callback=self._goal,
                cancel_callback=self._cancel, callback_group=self._group)
            self._timer = self.create_timer(1. / rate, self._tick, callback_group=self._group)
            self._diagnostic_timer = self.create_timer(1., self._diagnostics,
                                                       callback_group=self._group)
            self.add_on_set_parameters_callback(self._check_parameters)
        except BaseException:
            self.engine.shutdown(return_home=False)
            raise
        mode = 'SIMULATION' if simulate else 'HARDWARE'
        self.get_logger().info(f'{mode}; selected arms={selection}; motor coordinates on /t701_ht/*')

    def _check_parameters(self, parameters):
        controlled = {'simulate', 'arms', 'enable_on_start', 'state_rate',
                      'command_duration', 'description_dir'}
        return SetParametersResult(successful=not any(p.name in controlled for p in parameters),
                                   reason='restart the driver to change its configuration')

    def _report(self, error):
        now = time.monotonic()
        if now - self._last_error_log >= 1.:
            self.get_logger().error(str(error))
            self._last_error_log = now

    def _tick(self):
        with self._lock:
            if self._closing:
                return
            try:
                self.engine.tick()
                q, v, effort = self.engine.state()
                stamp = self.get_clock().now().to_msg()
                motor = JointState()
                motor.header.stamp = stamp
                motor.name = list(self.model.names)
                motor.position = np.asarray(q, float).tolist()
                motor.velocity = np.asarray(v, float).tolist()
                motor.effort = np.asarray(effort, float).tolist()
                self._motor_publisher.publish(motor)
                names, position, velocity = self.model.joint_states(q, v)
                state = JointState()
                state.header.stamp = stamp
                state.name = list(names)
                state.position = np.asarray(position, float).tolist()
                state.velocity = np.asarray(velocity, float).tolist()
                self._joint_publisher.publish(state)
                for side, transform in self.model.forward(q).items():
                    pose = PoseStamped()
                    pose.header.stamp = stamp
                    pose.header.frame_id = 'base_link'
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
                        float(value) for value in transform[:3, 3])
                    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
                    (pose.pose.orientation.x, pose.pose.orientation.y,
                     pose.pose.orientation.z, pose.pose.orientation.w) = (
                        float(value) for value in quaternion)
                    self._pose_publishers[side].publish(pose)
            except Exception as error:
                self._report(error)
                try:
                    self.engine.stop()
                except Exception as stop_error:
                    self._report(stop_error)

    def _diagnostics(self):
        with self._lock:
            if self._closing:
                return
            info = dict(self.engine.status)
            info['backend'] = self.backend_mode
        state = DiagnosticStatus()
        state.name = 't701_ht_driver'
        state.hardware_id = 'T701-HT'
        error = str(info.get('last_error') or '')
        state.level = DiagnosticStatus.ERROR if error else (
            DiagnosticStatus.WARN if info.get('collision_blocked') or not info.get('enabled')
            else DiagnosticStatus.OK)
        state.message = error or str(info.get('mode', 'idle'))
        state.values = [KeyValue(key=str(k), value=str(v)) for k, v in info.items()]
        report = DiagnosticArray()
        report.header.stamp = self.get_clock().now().to_msg()
        report.status = [state]
        self._diagnostics_publisher.publish(report)

    def _joint_command(self, message):
        with self._lock:
            try:
                if self._goal_pending or self._closing:
                    raise ValueError('an action is active or the driver is closing')
                if message.effort:
                    raise ValueError('effort commands are unsupported')
                if message.header.stamp.sec or message.header.stamp.nanosec:
                    raise ValueError('joint commands use immediate execution; header stamp must be zero')
                point = {'time_from_start': self.command_duration, 'positions': list(message.position)}
                if message.velocity:
                    point['velocities'] = list(message.velocity)
            except Exception as error:
                self._report(error)
                return
        try:
            self.engine.submit_trajectory(list(message.name), [point])
        except Exception as error:
            self._report(error)

    def _gripper_command(self, side, message):
        with self._lock:
            try:
                if self._closing:
                    return
                self.engine.gripper(side, float(message.data))
            except Exception as error:
                self._report(error)

    def _target_pose(self, side, message):
        with self._lock:
            try:
                if self._closing or self._goal_pending:
                    raise ValueError('an action is active or the driver is closing')
                if side not in self.model.sides:
                    raise ValueError('target arm is not selected')
                self.engine.submit_pose(side, pose_transform(message))
            except Exception as error:
                self._report(error)

    def _home(self, _request, response):
        try:
            self.engine.home()
            response.success = True
            response.message = 'Smooth return accepted; monitor motor_states and diagnostics for arrival.'
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def _stop(self, _request, response):
        with self._lock:
            try:
                self.engine.stop()
                response.success = True
                response.message = 'Damping brake requested; this service does not force a return path.'
            except Exception as error:
                response.success = False
                response.message = str(error)
        return response

    def _set_enabled(self, request, response):
        with self._lock:
            try:
                self.engine.set_enabled(bool(request.data))
                response.success = True
                response.message = 'Enabled.' if request.data else 'Disabled with damping brake.'
            except Exception as error:
                response.success = False
                response.message = str(error)
        return response

    def _goal(self, request):
        with self._lock:
            try:
                if self._closing or self._goal_pending:
                    raise ValueError('driver closing or another trajectory action is active')
                if not self.engine.status.get('enabled', False):
                    raise ValueError('enable the driver before sending trajectories')
                if self.engine.status.get('active_id') is not None:
                    raise ValueError('another command or homing operation is active')
                trajectory_points(request, self.model.names)
                self._goal_pending = True
                return GoalResponse.ACCEPT
            except Exception as error:
                self._report(error)
                return GoalResponse.REJECT

    def _cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _execute_goal(self, handle):
        result = FollowJointTrajectory.Result()
        canceled = False
        token = None
        try:
            with self._lock:
                self._active_goal = handle
                if self._closing:
                    raise RuntimeError('driver is closing')
                names, points = trajectory_points(handle.request, self.model.names)
            token = self.engine.submit_trajectory(names, points)
            feedback_order = [list(self.model.names).index(name) for name in names]
            while rclpy.ok() and not self._closing:
                if handle.is_cancel_requested and not canceled:
                    self.engine.cancel(token, return_home=True)
                    canceled = True
                with self._lock:
                    outcome = self.engine.result(token)
                    q, v, _ = self.engine.state()
                feedback = FollowJointTrajectory.Feedback()
                feedback.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_names = names
                feedback.actual.positions = np.asarray(q, float)[feedback_order].tolist()
                feedback.actual.velocities = np.asarray(v, float)[feedback_order].tolist()
                handle.publish_feedback(feedback)
                if outcome is not None:
                    result.error_string = str(outcome.get('error') or '')
                    if canceled:
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        handle.canceled()
                    elif outcome.get('success', False):
                        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                        handle.succeed()
                    else:
                        result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                        handle.abort()
                    return result
                time.sleep(.05)
            raise RuntimeError('driver shutting down; trajectory was not completed')
        except Exception as error:
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = str(error)
            self._report(error)
            if handle.is_active:
                handle.abort()
            return result
        finally:
            with self._lock:
                self._active_goal = None
                self._goal_pending = False

    def close(self):
        with self._lock:
            if self._closing:
                return
            self._closing = True
            self._timer.cancel()
            self._diagnostic_timer.cancel()
            self.get_logger().info('Closing: return home on valid feedback, then apply damping brake.')
        self.engine.shutdown(return_home=True)


def main(args=None):
    # Keep DDS alive while orderly shutdown returns the arm to zero. The default
    # rclpy signal hook closes the context before our motor cleanup can complete.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    received = 0
    def interrupt_driver(_signum, _frame):
        nonlocal received
        received += 1
        if received > 1 and node is not None:
            # A second operator interruption must never restart an automatic path.
            node.engine.stop()
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, interrupt_driver)
                for sig in (signal.SIGINT, signal.SIGTERM)}
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = DriverNode()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    executor.shutdown(timeout_sec=2.)
                    node.destroy_node()
            else:
                executor.shutdown(timeout_sec=2.)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == '__main__':
    main()
