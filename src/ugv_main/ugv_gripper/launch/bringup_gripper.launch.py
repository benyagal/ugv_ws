from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    roller_channel_arg = DeclareLaunchArgument('roller_channel', default_value='0')
    left_channel_arg = DeclareLaunchArgument('left_channel', default_value='1')
    right_channel_arg = DeclareLaunchArgument('right_channel', default_value='2')
    relay1_pin_arg = DeclareLaunchArgument('relay1_pin', default_value='29')
    relay2_pin_arg = DeclareLaunchArgument('relay2_pin', default_value='31')
    home_switch_pin_arg = DeclareLaunchArgument('home_switch_pin', default_value='33')
    # PCA9685 is wired to physical pins 27 (SDA) / 28 (SCL), not the
    # default/primary I2C bus - adjust i2c_bus if this carrier board maps
    # those pins to a different /dev/i2c-N than 0.
    i2c_bus_arg = DeclareLaunchArgument('i2c_bus', default_value='0')
    i2c_address_arg = DeclareLaunchArgument('i2c_address', default_value='64')  # 0x40
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
    #
    # NOTE: the servos moved to a PCA9685 (I2C, pins 27/28) - I2C pins are
    # fixed-function on this SoC (not GPIO-muxable), so no devmem fixup like
    # this is expected to be needed for them. If PCA9685 communication fails
    # at startup, that's the first thing to double check though.
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
            'roller_channel': LaunchConfiguration('roller_channel'),
            'left_channel': LaunchConfiguration('left_channel'),
            'right_channel': LaunchConfiguration('right_channel'),
            'relay1_pin': LaunchConfiguration('relay1_pin'),
            'relay2_pin': LaunchConfiguration('relay2_pin'),
            'home_switch_pin': LaunchConfiguration('home_switch_pin'),
            'i2c_bus': LaunchConfiguration('i2c_bus'),
            'i2c_address': LaunchConfiguration('i2c_address'),
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
        roller_channel_arg,
        left_channel_arg,
        right_channel_arg,
        relay1_pin_arg,
        relay2_pin_arg,
        home_switch_pin_arg,
        i2c_bus_arg,
        i2c_address_arg,
        with_joy_arg,
        fix_relay1_pinmux,
        fix_relay2_pinmux,
        delayed_gripper_node,
        gripper_joy_node,
    ])
