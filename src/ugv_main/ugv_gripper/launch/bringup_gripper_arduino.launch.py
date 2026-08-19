from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')
    baud_rate_arg = DeclareLaunchArgument('baud_rate', default_value='115200')
    with_joy_arg = DeclareLaunchArgument('with_joy', default_value='true')

    gripper_arduino_node = Node(
        package='ugv_gripper',
        executable='gripper_arduino_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
        }],
        output='screen',
    )

    gripper_joy_node = Node(
        package='ugv_gripper',
        executable='gripper_joy_ctrl',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_joy')),
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        with_joy_arg,
        gripper_arduino_node,
        gripper_joy_node,
    ])
