"""Default to a simulated dual arm; no motor SDK is imported in simulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    description = Path(get_package_share_directory('t701_ht_description'))
    sdk = Path(get_package_share_directory('t701_ht_ros2'))
    robot_description = (description / 'urdf' / 'T701-HT.urdf').read_text()
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),
        Node(package='t701_ht_ros2', executable='driver', output='screen',
             parameters=[{
                 'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool),
                 'arms': LaunchConfiguration('arms'),
                 'enable_on_start': ParameterValue(LaunchConfiguration('enable_on_start'), value_type=bool),
                 'state_rate': ParameterValue(LaunchConfiguration('state_rate'), value_type=float),
                 'description_dir': str(description),
             }]),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(sdk / 'config' / 't701_ht.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('simulate', default_value='true',
                              description='true: virtual backend; false: physical CAN motors'),
        DeclareLaunchArgument('arms', default_value='dual',
                              choices=['left', 'right', 'dual']),
        DeclareLaunchArgument('enable_on_start', default_value='false',
                              description='Hardware only: explicitly enable motion at startup'),
        DeclareLaunchArgument('state_rate', default_value='50.0'),
        DeclareLaunchArgument('rviz', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
