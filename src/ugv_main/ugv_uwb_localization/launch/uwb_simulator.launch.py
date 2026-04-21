from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation clock if true'),
        DeclareLaunchArgument('source_topic', default_value='/odometry/local', description='Odometry topic used as simulator source'),
        DeclareLaunchArgument('pose_topic', default_value='/uwb/pose', description='Pose topic published by the simulator'),
        DeclareLaunchArgument('status_topic', default_value='/uwb/status', description='Status topic published by the simulator'),
        Node(
            package='ugv_uwb_localization',
            executable='uwb_simulator',
            name='uwb_simulator',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'source_topic': LaunchConfiguration('source_topic')},
                {'pose_topic': LaunchConfiguration('pose_topic')},
                {'status_topic': LaunchConfiguration('status_topic')},
            ],
        ),
    ])