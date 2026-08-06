import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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

    uwb_serial_port_arg = DeclareLaunchArgument(
        'uwb_serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the DWM1001C tag device'
    )

    uwb_tag_id_arg = DeclareLaunchArgument(
        'uwb_tag_id',
        default_value='uwb_tag',
        description='DWM1001 tag ID label (used for topic name matching)'
    )

    use_map_server_arg = DeclareLaunchArgument(
        'use_map_server',
        default_value='true',
        description='Whether to serve the pre-built 2D map for RViz visualization (no AMCL - visual reference only)'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('ugv_nav'), 'maps', 'map.yaml'),
        description='Path to the pre-built 2D map YAML file'
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
            # Magnetometer ruled out (dominated by the robot's own motors/
            # chassis, confirmed non-monotonic during a clean rotation).
            # Switching to pure gyro yaw instead: the earlier "fast runaway
            # rotation" turned out to be a SEPARATE bug (ekf_global's yaw
            # state being completely unconstrained), not the gyro itself -
            # now that that's fixed, gyro-only yaw should just show its
            # normal slow bias drift (as seen in the very first re-enable
            # test) rather than a catastrophic spin. Give it a correct
            # starting heading once via RViz's "2D Pose Estimate" tool
            # (publishes /initialpose, which the EKF's set_pose picks up)
            # after launch, since the gyro can only track RELATIVE
            # rotation from wherever it starts.
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

    # ─── Real DWM1001C hardware nodes (use_uwb_sim=false) ─────────────────────
    #
    # Pipeline:
    #   active_tag  → /uwb/point_raw   (PointStamped, dwm1001 frame)
    #   dwm1001_transform → /uwb/odometry  (Odometry, map frame)
    #   uwb_driver  → /uwb/pose        (PoseWithCovarianceStamped, map frame)
    #
    # The static TF dwm1001→map is identity because the anchor origó (0,0)
    # matches the map frame origó (SW istálló sarok).
    # ──────────────────────────────────────────────────────────────────────────

    uwb_static_tf_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_uwb_sim')),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dwm1001_to_map_tf',
        # Recalibrated using 4 reference points measured via RViz "Publish
        # Point" (map frame) + /uwb/point_raw (dwm1001 frame), fitted with
        # least squares (2D Procrustes). Residual error ~9-17cm across all
        # 4 points - point 1 was remeasured after letting the UWB position
        # settle (previous quick reading was inaccurate), and a 4th point
        # was added for extra robustness.
        # x, y, z, yaw, pitch, roll, parent_frame, child_frame
        arguments=['0.834', '-1.207', '0', '1.739', '0', '0', 'map', 'dwm1001'],
        output='screen',
    )

    uwb_active_tag_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_uwb_sim')),
        package='dwm1001_driver',
        executable='active_tag',
        name='active_tag',
        output='screen',
        parameters=[
            {'serial_port': LaunchConfiguration('uwb_serial_port')},
            {'tag_id': LaunchConfiguration('uwb_tag_id')},
            {'samples': 3},
        ],
        remappings=[
            ('~/output/' + 'uwb_tag', '/uwb/point_raw'),
        ],
    )

    uwb_transform_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_uwb_sim')),
        package='dwm1001_transform',
        executable='dwm1001_transform',
        name='dwm1001_transform',
        output='screen',
        parameters=[
            # More realistic than the initial 1-2cm lab estimate: with only
            # 4 anchors in a small workspace, real accuracy is closer to
            # 5-10cm. A too-tight covariance made the EKF's outlier rejection
            # (pose0_rejection_threshold) throw away most real corrections.
            # Layout: [xx, xy, xz, yx, yy, yz, zx, zy, zz]
            {'position_cov': [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]},
        ],
        remappings=[
            ('input/tag_position', '/uwb/point_raw'),
            ('~/output/odometry/ips', '/uwb/odometry'),
        ],
    )

    uwb_driver_node = Node(
        condition=UnlessCondition(LaunchConfiguration('use_uwb_sim')),
        package='ugv_uwb_localization',
        executable='uwb_driver',
        name='uwb_driver_node',
        output='screen',
        parameters=[
            {'input_topic': '/uwb/odometry'},
            {'pose_topic': '/uwb/pose'},
            {'status_topic': '/uwb/status'},
            {'frame_id': 'map'},
        ],
    )

    # Pre-built 2D map, served purely for RViz visual reference (no AMCL -
    # localization still comes entirely from the UWB+odometry EKF chain above).
    map_server_node = Node(
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'yaml_filename': LaunchConfiguration('map')},
        ]
    )

    lifecycle_manager_map_node = Node(
        condition=IfCondition(LaunchConfiguration('use_map_server')),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'autostart': True},
            {'node_names': ['map_server']},
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
        uwb_serial_port_arg,
        uwb_tag_id_arg,
        use_map_server_arg,
        map_arg,
        robot_state_launch,
        bringup_node,
        imu_complementary_filter_node,
        driver_node,
        base_node,
        local_ekf_node,
        # UWB: simulator OR real hardware (mutually exclusive)
        uwb_simulator_node,
        uwb_static_tf_node,
        uwb_active_tag_node,
        uwb_transform_node,
        uwb_driver_node,
        global_ekf_node,
        map_server_node,
        lifecycle_manager_map_node,
    ])