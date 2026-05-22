"""
UWB-alapú navigáció Gazebo szimulációhoz, TEB local plannerrel.

Indítás:
  Terminal 1: ros2 launch ugv_gazebo bringup.launch.py
  Terminal 2: ros2 launch ugv_gazebo nav_uwb.launch.py

Architektúra:
  Gazebo diff-drive  →  /odom  +  odom→base_footprint TF
  /odom  →  UWB szimulátor  →  /uwb/pose
  /odom + /uwb/pose  →  global EKF  →  map→odom TF
  map_server  →  statikus térkép (costmap static layer)
  Nav2 (TEB, navigation_launch.py, AMCL nélkül)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Argumentumok
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(ugv_gazebo_dir, 'maps', 'map.yaml'),
        description='A már felépített 2D-s térkép YAML fájljának elérési útja'
    )

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(ugv_gazebo_dir, 'param', 'uwb_teb_gazebo.yaml'),
        description='Nav2 paraméter fájl (TEB local planner, lidar nélkül)'
    )

    # map_server — a felépített 2D-s térképet szolgáltatja a Nav2 global costmap
    # static_layer-jéhez. AMCL helyett a global EKF adja a lokalizációt.
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': LaunchConfiguration('map')},
        ]
    )

    # A map_server lifecycle node, hogy automatikusan elinduljon.
    lifecycle_manager_map_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': ['map_server']},
        ]
    )

    # UWB szimulátor — a Gazebo /odom topicot olvassa "igazságként",
    # zajt, kieséseket és outliereket injektál, és /uwb/pose-ra publikál.
    # Valódi hardvernél ez a node cserélődik ki a driver node-ra.
    uwb_simulator_node = Node(
        package='ugv_uwb_localization',
        executable='uwb_simulator',
        name='uwb_simulator',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'source_topic': '/odom'},
            {'pose_topic': '/uwb/pose'},
            {'status_topic': '/uwb/status'},
            {'frame_id': 'map'},
            # 1Hz: az EKF csak másodpercenként kap UWB korrekciót.
            # Ritkább korrekció = simább mozgás RViz-ben.
            {'publish_rate': 1.0},
            # 0.02m tényleges zaj: az EKF max ±2cm-t korrigál mérésenként.
            # A variancia (0.02²=0.0004) kicsi marad, ezért az outlier szűrés
            # (Mahalanobis = 1.0/0.02 = 50 >> 5.0 küszöb) teljesen működik.
            {'position_noise_stddev': 0.02},
            # 0.5% outlier valószínűség: ritkábban keletkezik nagy ugrás.
            {'outlier_probability': 0.005},
        ]
    )

    # Global EKF — /odom (Gazebo) + /uwb/pose → map→odom TF
    # A Gazebo diff-drive plugin már adja az odom→base_footprint TF-et,
    # ezért itt nincs local EKF réteg, nem lesz TF ütközés.
    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_filter_node',
        output='screen',
        parameters=[
            os.path.join(ugv_bringup_dir, 'param', 'ekf_global_gazebo.yaml'),
        ],
        remappings=[('/odometry/filtered', '/odometry/global')]
    )

    # Nav2 navigációs stack — csak a navigation_launch.py, AMCL nélkül.
    # A lokalizációt (map→odom TF) a global EKF adja.
    # A /scan topicot a Gazebo szimulált lidarja adja az akadálykerüléshez.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': 'true',
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(ugv_gazebo_dir, 'rviz', 'view_nav_2d.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    robot_pose_publisher_node = Node(
        package='robot_pose_publisher',
        executable='robot_pose_publisher',
        name='robot_pose_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': True},
            {'is_stamped': True},
            {'map_frame': 'map'},
            {'base_frame': 'base_footprint'},
        ]
    )

    return LaunchDescription([
        map_arg,
        params_arg,
        map_server_node,
        lifecycle_manager_map_node,
        uwb_simulator_node,
        global_ekf_node,
        nav2_navigation_launch,
        rviz2_node,
        robot_pose_publisher_node,
    ])
