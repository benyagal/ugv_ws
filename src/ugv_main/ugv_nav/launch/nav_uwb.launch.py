import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_navigation(context):
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = LaunchConfiguration('params_file').perform(context)

    if LaunchConfiguration('uwb_heading_correction').perform(context).lower() == 'true':
        with open(params_file) as f:
            params = yaml.safe_load(f)
        bt_params = params['bt_navigator']['ros__parameters']
        bt_params['default_nav_to_pose_bt_xml'] = os.path.join(
            get_package_share_directory('ugv_nav_bt'), 'behavior_trees',
            'navigate_to_pose_heading_calibration.xml')
        bt_params['plugin_lib_names'] = (
            bt_params['plugin_lib_names'] + ['ugv_heading_calibration_bt_node'])
        with tempfile.NamedTemporaryFile(
                'w', prefix='uwb_teb_heading_', suffix='.yaml', delete=False) as f:
            yaml.safe_dump(params, f)
            params_file = f.name

    # Stock Nav2 navigation servers (controller_server/smoother_server/
    # planner_server/behavior_server/bt_navigator/waypoint_follower/
    # velocity_smoother/lifecycle_manager_navigation). Deliberately reusing
    # nav2_bringup's own navigation_launch.py as-is instead of a local copy -
    # it already excludes AMCL/map_server, which is exactly what we want
    # since both are handled by bringup_localization_uwb.launch.py above.
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': LaunchConfiguration('autostart'),
            'params_file': params_file,
            # navigation_launch.py evaluates this via PythonExpression(['not
            # ', use_composition]) i.e. a real Python eval() - it must be the
            # capitalized Python literal 'False', not lowercase 'false'
            # (which raised "name 'false' is not defined").
            'use_composition': 'False',
        }.items()
    )]


def generate_launch_description():
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')

    default_map_path = os.path.join(ugv_nav_dir, 'maps', 'map.yaml')
    default_params_path = os.path.join(ugv_nav_dir, 'param', 'uwb_teb.yaml')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz2'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='nav_2d',
        description='Which RViz configuration to use (see ugv_description display.launch.py) - '
                     'defaults to the top-down 2D nav view with the map displayed, not the '
                     'generic 3D bringup view'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    use_uwb_sim_arg = DeclareLaunchArgument(
        'use_uwb_sim',
        default_value='true',
        description='Start the UWB pose simulator instead of the real DWM1001 hardware'
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the pre-built static map yaml file'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_path,
        description='Full path to the Nav2 parameters file (TEB local planner, UWB+IMU localization, no LIDAR)'
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start the Nav2 lifecycle nodes'
    )

    uwb_heading_correction_arg = DeclareLaunchArgument(
        'uwb_heading_correction',
        default_value='false',
        description='Correct the yaw from UWB on straight segments, and drive a short '
                    'straight calibration segment during navigation when none was found'
    )

    use_gyro_angular_feedback_arg = DeclareLaunchArgument(
        'use_gyro_angular_feedback',
        default_value='true',
        description='ugv_driver: close the turn-rate loop on the IMU gyro instead of the wheel encoders'
    )

    stall_detection_arg = DeclareLaunchArgument(
        'stall_detection',
        default_value='true',
        description='ugv_driver: stop the motors when a commanded turn makes no rotation at full duty'
    )

    # Sensors + IMU/UWB EKF localization (map->odom TF) + static map_server
    # (no AMCL - the map is only used here to feed the costmaps' static
    # layer, not for scan-matching localization). No LIDAR is started here.
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_bringup_dir, 'launch', 'bringup_localization_uwb.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_uwb_sim': LaunchConfiguration('use_uwb_sim'),
            'map': LaunchConfiguration('map'),
            'use_map_server': 'true',
            'uwb_heading_correction': LaunchConfiguration('uwb_heading_correction'),
            'use_gyro_angular_feedback': LaunchConfiguration('use_gyro_angular_feedback'),
            'stall_detection': LaunchConfiguration('stall_detection'),
        }.items()
    )

    return LaunchDescription([
        use_rviz_arg,
        rviz_config_arg,
        use_sim_time_arg,
        use_uwb_sim_arg,
        map_arg,
        params_file_arg,
        autostart_arg,
        uwb_heading_correction_arg,
        use_gyro_angular_feedback_arg,
        stall_detection_arg,
        localization_launch,
        OpaqueFunction(function=launch_navigation),
    ])
