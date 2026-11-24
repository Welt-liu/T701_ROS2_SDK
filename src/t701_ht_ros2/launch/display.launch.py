"""Visualize the robot with the same collision-checked simulated driver."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path = Path(get_package_share_directory('t701_ht_ros2')) / 'launch' / 'sdk.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('arms', default_value='dual', choices=['left', 'right', 'dual']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),
                                 launch_arguments={'simulate': 'true', 'rviz': 'true',
                                                   'arms': LaunchConfiguration('arms')}.items()),
    ])
