from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    i2c_bus_arg = DeclareLaunchArgument('i2c_bus', default_value='7')
    pca9685_address_arg = DeclareLaunchArgument('pca9685_address', default_value='66')  # 0x42
    relay1_pin_arg = DeclareLaunchArgument('relay1_pin', default_value='29')
    relay2_pin_arg = DeclareLaunchArgument('relay2_pin', default_value='31')
    home_switch_pin_arg = DeclareLaunchArgument('home_switch_pin', default_value='33')
    with_joy_arg = DeclareLaunchArgument('with_joy', default_value='true')

    gripper_node = Node(
        package='ugv_gripper',
        executable='gripper_node',
        parameters=[{
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'pca9685_address': LaunchConfiguration('pca9685_address'),
            'relay1_pin': LaunchConfiguration('relay1_pin'),
            'relay2_pin': LaunchConfiguration('relay2_pin'),
            'home_switch_pin': LaunchConfiguration('home_switch_pin'),
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
        i2c_bus_arg,
        pca9685_address_arg,
        relay1_pin_arg,
        relay2_pin_arg,
        home_switch_pin_arg,
        with_joy_arg,
        gripper_node,
        gripper_joy_node,
    ])
