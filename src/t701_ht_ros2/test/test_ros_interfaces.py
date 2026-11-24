"""Validate ROS message boundaries without constructing a backend or opening CAN."""

import math
import os
from pathlib import Path
import importlib.util
import subprocess
from types import SimpleNamespace
from unittest.mock import Mock
import threading

import pytest
import numpy as np
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from rclpy.action import GoalResponse
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from t701_ht_ros2.demo_node import DemoNode, point_at
from t701_ht_ros2.driver_node import trajectory_points
from t701_ht_ros2.driver_node import DriverNode
from t701_ht_ros2.driver_node import pose_transform


def request():
    result = FollowJointTrajectory.Goal()
    result.trajectory.joint_names = ['right_joint2', 'right_joint1']
    result.trajectory.points = [point_at(0., [0., 0.]), point_at(3., [.1, -.1])]
    return result


def test_action_preserves_requested_order_and_times():
    names, points = trajectory_points(request(), ('right_joint1', 'right_joint2'))
    assert names == ['right_joint2', 'right_joint1']
    assert points[0]['time_from_start'] == 0.
    assert points[1]['time_from_start'] == 3.
    assert points[1]['positions'] == [.1, -.1]


@pytest.mark.parametrize('change', [
    lambda r: setattr(r.trajectory, 'joint_names', ['right_joint1', 'right_joint1']),
    lambda r: setattr(r.trajectory, 'joint_names', ['left_joint1', 'right_joint1']),
    lambda r: setattr(r.trajectory.points[1], 'positions', [.1]),
    lambda r: setattr(r.trajectory.points[1], 'positions', [math.nan, .1]),
    lambda r: setattr(r.trajectory.points[1], 'velocities', [.1]),
    lambda r: setattr(r.trajectory.points[1], 'effort', [.1, .1]),
    lambda r: setattr(r.trajectory.points[1], 'accelerations', [.1, .1]),
    lambda r: setattr(r.trajectory.header.stamp, 'sec', 1),
    lambda r: setattr(r.goal_time_tolerance, 'sec', 1),
    lambda r: setattr(r, 'goal_tolerance', [JointTolerance(name='right_joint1')]),
    lambda r: setattr(r.trajectory.points[1].time_from_start, 'sec', 0),
    lambda r: setattr(r.multi_dof_trajectory, 'joint_names', ['floating']),
])
def test_unsupported_or_malformed_action_is_rejected(change):
    goal = request()
    change(goal)
    with pytest.raises(ValueError):
        trajectory_points(goal, ('right_joint1', 'right_joint2'))


def test_missing_velocities_are_valid_and_not_invented():
    goal = request()
    for point in goal.trajectory.points:
        point.velocities = []
    _, points = trajectory_points(goal, ('right_joint1', 'right_joint2'))
    assert all('velocities' not in point for point in points)


def test_launch_entry_points_match_package_scripts(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros_logs'))
    package = Path(__file__).parents[1]
    text = (package / 'launch' / 'sdk.launch.py').read_text()
    assert "executable='driver'" in text
    assert "default_value='true'" in text
    assert "'simulate': ParameterValue" in text
    spec = importlib.util.spec_from_file_location('test_sdk_launch', package / 'launch' / 'sdk.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declarations = {entry.name for entry in description.entities if hasattr(entry, 'name')}
    assert {'simulate', 'arms', 'enable_on_start', 'state_rate', 'rviz'} <= declarations


def test_demo_default_only_reads_state():
    from t701_ht_ros2.demo_node import parser
    options = parser().parse_args([])
    assert options.mode == 'state'
    assert options.amplitude == .1


def _entry_dispatch(arguments, environment=None):
    """Run the actual shell dispatch, stopping before ROS or hardware loading."""
    workspace = Path(__file__).resolve().parents[3]
    text = (workspace / 'start.sh').read_text()
    dispatch = text.split('unset PYTHONPATH PYTHONHOME LD_PRELOAD', 1)[0]
    probe = '\nprintf "PROBE_DOMAIN=%s\\n" "$ROS_DOMAIN_ID"\nprintf "PROBE_ARG=%s\\n" "$@"\n'
    env = {key: value for key, value in os.environ.items()
           if key not in ('T701_HT_SIM_DOMAIN_ID', 'T701_HT_REAL_DOMAIN_ID', 'ROS_DOMAIN_ID')}
    env.update(environment or {})
    process = subprocess.run(['bash', '-s', '--', *arguments],
                             input=dispatch + probe, text=True, capture_output=True,
                             cwd=workspace, env=env, timeout=3.)
    domain, forwarded = None, []
    for line in process.stdout.splitlines():
        if line.startswith('PROBE_DOMAIN='):
            domain = line.partition('=')[2]
        elif line.startswith('PROBE_ARG='):
            forwarded.append(line.partition('=')[2])
    return process, domain, forwarded


@pytest.mark.parametrize('mode,domain,simulate', [('sim', '20', 'true'),
                                               ('real', '21', 'false')])
def test_mode_entries_select_distinct_domains_and_backends(mode, domain, simulate):
    process, actual_domain, arguments = _entry_dispatch([mode], {'ROS_DOMAIN_ID': '99'})
    assert process.returncode == 0, process.stderr
    assert actual_domain == domain
    assert arguments == ['launch', 't701_ht_ros2', 'sdk.launch.py',
                         'simulate:=' + simulate, 'rviz:=true']


def test_no_argument_entry_uses_the_same_isolated_simulation_domain():
    process, domain, arguments = _entry_dispatch([], {'ROS_DOMAIN_ID': '21'})
    assert process.returncode == 0, process.stderr
    assert domain == '20'
    assert arguments == ['launch', 't701_ht_ros2', 'sdk.launch.py',
                         'simulate:=true', 'rviz:=true']


def test_mode_entry_preserves_allowed_hardware_options():
    options = ['arms:=right', 'rviz:=false', 'state_rate:=40', 'enable_on_start:=false']
    process, domain, arguments = _entry_dispatch(['real', *options], {
        'T701_HT_SIM_DOMAIN_ID': '30', 'T701_HT_REAL_DOMAIN_ID': '31'})
    assert process.returncode == 0, process.stderr
    assert domain == '31'
    assert arguments[-len(options):] == options
    assert 'simulate:=false' in arguments


@pytest.mark.parametrize('environment', [
    {'T701_HT_SIM_DOMAIN_ID': '21'},
    {'T701_HT_SIM_DOMAIN_ID': '021', 'T701_HT_REAL_DOMAIN_ID': '21'},
    {'T701_HT_REAL_DOMAIN_ID': '-1'},
    {'T701_HT_REAL_DOMAIN_ID': '233'},
    {'T701_HT_REAL_DOMAIN_ID': 'text'},
])
def test_mode_entry_rejects_overlapping_or_invalid_domains(environment):
    process, _, arguments = _entry_dispatch(['sim'], environment)
    assert process.returncode == 2
    assert not arguments


@pytest.mark.parametrize('arguments', [
    ['sim', 'simulate:=false'],
    ['real', 'simulate:=true'],
    ['real', 'launch', 't701_ht_ros2', 'display.launch.py'],
    ['real', 'run', 't701_ht_ros2', 'driver'],
    ['real', 'demo', '--target', 'sim'],
    ['sim', 'demo', '--target=real'],
])
def test_mode_entry_does_not_allow_backend_or_demo_target_overrides(arguments):
    process, _, forwarded = _entry_dispatch(arguments)
    assert process.returncode == 2
    assert not forwarded


@pytest.mark.parametrize('mode', ['sim', 'real'])
def test_mode_demo_dispatch_injects_the_required_target(mode):
    process, _, arguments = _entry_dispatch([mode, 'demo', '--mode', 'joint', '--duration', '5'])
    assert process.returncode == 0, process.stderr
    assert arguments == ['run', 't701_ht_ros2', 'demo', '--target', mode,
                         '--mode', 'joint', '--duration', '5']


@pytest.mark.parametrize('operation', [
    ['topic', 'echo', '/diagnostics', '--once'],
    ['service', 'call', '/t701_ht/home', 'std_srvs/srv/Trigger', '{}'],
    ['action', 'info', '/t701_ht/follow_joint_trajectory'],
    ['run', 'rviz2', 'rviz2'],
])
def test_mode_cli_remains_in_the_selected_domain(operation):
    process, domain, arguments = _entry_dispatch(['real', *operation])
    assert process.returncode == 0, process.stderr
    assert domain == '21' and arguments == operation


def test_demo_rejects_abbreviated_target_override():
    from t701_ht_ros2.demo_node import parser
    with pytest.raises(SystemExit):
        parser().parse_args(['--target', 'real', '--ta', 'sim'])


@pytest.mark.parametrize('expected,actual', [('real', 'sim'), ('sim', 'real')])
def test_demo_requires_matching_diagnostic_backend(monkeypatch, expected, actual):
    import t701_ht_ros2.demo_node as demo
    monkeypatch.setattr(demo.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(demo.rclpy, 'spin_once', lambda *args, **kwargs: None)
    host = SimpleNamespace(last_diagnostics={'backend': actual}, get_logger=Mock())
    with pytest.raises(RuntimeError, match='no command sent'):
        DemoNode.require_backend(host, expected)


@pytest.mark.parametrize('target', ['sim', 'real'])
def test_demo_accepts_only_a_confirmed_matching_backend(monkeypatch, target):
    import t701_ht_ros2.demo_node as demo
    monkeypatch.setattr(demo.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(demo.rclpy, 'spin_once', lambda *args, **kwargs: None)
    logger = Mock()
    host = SimpleNamespace(last_diagnostics={'backend': target}, get_logger=lambda: logger)
    DemoNode.require_backend(host, target)
    logger.info.assert_called_once_with(f'Confirmed {target} driver.')


def test_demo_rejects_unconfirmed_backend_without_sending_a_command(monkeypatch):
    import t701_ht_ros2.demo_node as demo
    monkeypatch.setattr(demo.rclpy, 'ok', lambda: True)
    host = SimpleNamespace(last_diagnostics={'mode': 'idle'}, get_logger=Mock())
    with pytest.raises(RuntimeError, match='No confirmed real driver'):
        DemoNode.require_backend(host, 'real', seconds=0.)


def test_demo_main_sends_no_motion_when_backend_confirmation_fails(monkeypatch):
    import t701_ht_ros2.demo_node as demo
    host = SimpleNamespace(require_backend=Mock(side_effect=RuntimeError('backend mismatch')),
                           watch=Mock(), enable=Mock(), wait_home=Mock(), call_trigger=Mock(),
                           gripper=Mock(), ik=Mock(), motion=Mock(),
                           get_logger=Mock(), destroy_node=Mock())
    monkeypatch.setattr(demo, 'DemoNode', lambda: host)
    monkeypatch.setattr(demo.rclpy, 'init', Mock())
    monkeypatch.setattr(demo.rclpy, 'ok', lambda: True)
    shutdown = Mock()
    monkeypatch.setattr(demo.rclpy, 'shutdown', shutdown)
    monkeypatch.setattr(demo.signal, 'signal', Mock())
    with pytest.raises(SystemExit) as exit_info:
        demo.main(['--target', 'real', '--mode', 'joint'])
    assert exit_info.value.code == 1
    host.require_backend.assert_called_once_with('real')
    for name in ('watch', 'enable', 'wait_home', 'call_trigger', 'gripper', 'ik', 'motion'):
        getattr(host, name).assert_not_called()
    host.destroy_node.assert_called_once()
    shutdown.assert_called_once()


def test_diagnostics_identifies_the_actual_backend():
    from diagnostic_msgs.msg import DiagnosticArray
    host = SimpleNamespace(
        _lock=threading.RLock(), _closing=False, backend_mode='real',
        engine=SimpleNamespace(status={'mode': 'disabled', 'enabled': False}),
        _diagnostics_publisher=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            to_msg=lambda: DiagnosticArray().header.stamp)))
    DriverNode._diagnostics(host)
    report = host._diagnostics_publisher.publish.call_args.args[0]
    values = {entry.key: entry.value for entry in report.status[0].values}
    assert values['backend'] == 'real'
    assert values['mode'] == 'disabled'


def target_pose():
    result = PoseStamped()
    result.header.frame_id = 'base_link'
    result.pose.position.x = .4
    result.pose.position.y = .1
    result.pose.position.z = .3
    result.pose.orientation.w = 1.
    return result


def test_qp_pose_target_converts_base_frame_and_quaternion():
    transform = pose_transform(target_pose())
    np.testing.assert_allclose(transform[:3, 3], [.4, .1, .3])
    np.testing.assert_allclose(transform[:3, :3], np.eye(3))
    np.testing.assert_allclose(transform[3], [0., 0., 0., 1.])


@pytest.mark.parametrize('change', [
    lambda msg: setattr(msg.header, 'frame_id', ''),
    lambda msg: setattr(msg.header, 'frame_id', 'map'),
    lambda msg: setattr(msg.pose.position, 'x', math.nan),
    lambda msg: setattr(msg.pose.orientation, 'x', math.inf),
    lambda msg: setattr(msg.pose.orientation, 'w', 0.),
    lambda msg: setattr(msg.pose.orientation, 'w', 2.),
])
def test_ambiguous_or_invalid_qp_pose_is_rejected(change):
    message = target_pose()
    change(message)
    with pytest.raises(ValueError):
        pose_transform(message)


def test_pose_callback_delegates_only_to_shared_qp_engine():
    host = SimpleNamespace(
        _lock=threading.RLock(), _closing=False, _goal_pending=False,
        model=SimpleNamespace(sides=('left', 'right')),
        engine=SimpleNamespace(submit_pose=Mock()), _report=Mock())
    DriverNode._target_pose(host, 'right', target_pose())
    host.engine.submit_pose.assert_called_once()
    side, transform = host.engine.submit_pose.call_args.args
    assert side == 'right'
    np.testing.assert_allclose(transform, pose_transform(target_pose()))
    host._report.assert_not_called()


@pytest.mark.parametrize('side,pending,closing', [('right', True, False),
                                               ('right', False, True),
                                               ('left', False, False)])
def test_pose_callback_rejects_action_overlap_and_unselected_side(side, pending, closing):
    host = SimpleNamespace(
        _lock=threading.RLock(), _closing=closing, _goal_pending=pending,
        model=SimpleNamespace(sides=('right',)),
        engine=SimpleNamespace(submit_pose=Mock()), _report=Mock())
    DriverNode._target_pose(host, side, target_pose())
    host.engine.submit_pose.assert_not_called()
    host._report.assert_called_once()


def test_ik_demo_is_explicit_and_pose_distance_uses_orientation():
    from t701_ht_ros2.demo_node import parser, pose_distance
    options = parser().parse_args(['--mode', 'ik', '--side', 'right'])
    assert options.mode == 'ik' and options.side == 'right'
    first, second = target_pose(), target_pose()
    second.pose.position.z += .01
    assert pose_distance(first, second) == pytest.approx((.01, 0.))
    second.pose.orientation.z = 1.
    second.pose.orientation.w = 0.
    assert pose_distance(first, second)[1] == pytest.approx(math.pi)


@pytest.mark.parametrize('cancel_requested', [False, True])
def test_action_execution_maps_feedback_order_and_cancellation(monkeypatch, cancel_requested):
    import t701_ht_ros2.driver_node as driver
    monkeypatch.setattr(driver.rclpy, 'ok', lambda: True)
    goal = request()
    engine = SimpleNamespace(
        submit_trajectory=Mock(return_value=4),
        cancel=Mock(),
        result=Mock(return_value={'success': not cancel_requested, 'error': ''}),
        state=Mock(return_value=([.2, .1], [0., 0.], [0., 0.])),
    )
    host = SimpleNamespace(
        _lock=threading.RLock(), _closing=False, _goal_pending=True,
        _active_goal=None, engine=engine,
        model=SimpleNamespace(names=('right_joint1', 'right_joint2')),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: goal.trajectory.header.stamp)),
        _report=Mock(),
    )
    handle = SimpleNamespace(request=goal, is_cancel_requested=cancel_requested,
                             is_active=True, publish_feedback=Mock(), canceled=Mock(),
                             succeed=Mock(), abort=Mock())
    result = DriverNode._execute_goal(host, handle)
    assert result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
    feedback = handle.publish_feedback.call_args.args[0]
    assert feedback.joint_names == goal.trajectory.joint_names
    assert list(feedback.actual.positions) == [.1, .2]
    assert not host._goal_pending
    if cancel_requested:
        engine.cancel.assert_called_once_with(4, return_home=True)
        handle.canceled.assert_called_once()
        handle.succeed.assert_not_called()
    else:
        engine.cancel.assert_not_called()
        handle.succeed.assert_called_once()
