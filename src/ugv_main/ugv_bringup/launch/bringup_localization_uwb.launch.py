import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pub_odom_tf_arg = DeclareLaunchArgument(
        'pub_odom_tf',
        default_value='false',
        description='Whether the raw base node should publish odom TF'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='bringup',
        description='Choose which RViz configuration to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    start_base_stack_arg = DeclareLaunchArgument(
        'start_base_stack',
        default_value='true',
        description='Start the base serial bringup, driver and raw odometry nodes'
    )

    start_imu_filter_arg = DeclareLaunchArgument(
        'start_imu_filter',
        default_value='true',
        description='Start the complementary IMU filter that publishes /imu/data'
    )

    use_uwb_sim_arg = DeclareLaunchArgument(
        'use_uwb_sim',
        default_value='true',
        description='Start the UWB pose simulator'
    )

    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    )

    bringup_node = Node(
        condition=IfCondition(LaunchConfiguration('start_base_stack')),
        package='ugv_bringup',
        executable='ugv_bringup',
    )

    imu_complementary_filter_node = Node(
        condition=IfCondition(LaunchConfiguration('start_imu_filter')),
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ]
    )

    driver_node = Node(
        condition=IfCondition(LaunchConfiguration('start_base_stack')),
        package='ugv_bringup',
        executable='ugv_driver',
    )

    base_node = Node(
        condition=IfCondition(LaunchConfiguration('start_base_stack')),
        package='ugv_base_node',
        executable='base_node_ekf',
        parameters=[
            {'pub_odom_tf': LaunchConfiguration('pub_odom_tf')},
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('ugv_bringup'), 'param', 'ekf_local.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/odometry/filtered', '/odometry/local')]
    )

    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter_node',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('ugv_bringup'), 'param', 'ekf_global.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[('/odometry/filtered', '/odometry/global')]
    )

    uwb_simulator_node = Node(
        condition=IfCondition(LaunchConfiguration('use_uwb_sim')),
        package='ugv_uwb_localization',
        executable='uwb_simulator',
        name='uwb_simulator',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'source_topic': '/odometry/local'},
            {'pose_topic': '/uwb/pose'},
            {'status_topic': '/uwb/status'},
            {'frame_id': 'map'},
        ]
    )

    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        start_base_stack_arg,
        start_imu_filter_arg,
        use_uwb_sim_arg,
        robot_state_launch,
        bringup_node,
        imu_complementary_filter_node,
        driver_node,
        base_node,
        local_ekf_node,
        uwb_simulator_node,
        global_ekf_node,
    ])