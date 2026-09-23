#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # RViZ2 settings
  rviz2_config = os.path.join(
      get_package_share_directory('ldlidar'),
      'rviz',
      'view.rviz'
  )

  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_show_ld19',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_lidar_link'},
        # Stable by-id path (confirmed 2026-09-23): /dev/ttyACM0 is actually
        # the DWM1001 UWB tag's SEGGER J-Link VCOM, not the lidar - the ACM
        # index depends on USB enumeration order. The LD19 itself is the
        # other CDC-ACM device (1a86 "USB Single Serial").
        {'port_name': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5970075770-if00'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 225.0},
        {'angle_crop_max': 315.0}
      ]
  )

  # base_link to base_laser tf node
  base_footprint_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_laser_ld19',
    arguments=['0','0','0','0','0','0','base_footprint','base_lidar_link']
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_footprint_to_laser_tf_node)
  ld.add_action(rviz2_node)

  return ld
