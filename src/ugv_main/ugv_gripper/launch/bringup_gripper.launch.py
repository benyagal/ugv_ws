from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
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

    # On this carrier board (unrecognized by Jetson.GPIO - "not a Jetson
    # Developer Kit"), physical pins 29/31 (BOARD numbering) boot up with
    # their pinmux set to input, so GPIO.output() silently has no effect on
    # the actual pin voltage even though it reports success. NVIDIA's own
    # Jetson.GPIO library prints the fix for each affected pin (see
    # gpio_cdev.py's UserWarning); this only lasts until the next reboot, so
    # it must be reapplied on every boot before the relay GPIOs are used.
    # If relay1_pin/relay2_pin are ever changed, the devmem address for the
    # new pin must be re-derived the same way (run GPIO.setup(<pin>, OUT)
    # standalone and read the suggested "sudo busybox devmem ..." command
    # from the warning it prints).
    fix_relay1_pinmux = ExecuteProcess(
        cmd=['busybox', 'devmem', '0x2430068', 'w', '0x8'],  # physical pin 29
        output='screen',
    )

    fix_relay2_pinmux = ExecuteProcess(
        cmd=['busybox', 'devmem', '0x2430070', 'w', '0x8'],  # physical pin 31
        output='screen',
    )

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

    # Delay node startup slightly to guarantee the pinmux fixups above have
    # completed first.
    delayed_gripper_node = TimerAction(period=1.0, actions=[gripper_node])

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
        fix_relay1_pinmux,
        fix_relay2_pinmux,
        delayed_gripper_node,
        gripper_joy_node,
    ])
