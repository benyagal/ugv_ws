import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ugv_mission'), 'param', 'mission.yaml')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='ugv_mission',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='ugv_mission',
            executable='dock_server',
            name='dock_server',
            output='screen',
            parameters=[params],
        ),
    ])
