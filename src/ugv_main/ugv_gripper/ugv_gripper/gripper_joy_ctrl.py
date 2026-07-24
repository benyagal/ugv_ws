#!/usr/bin/env python3
"""Maps free joystick buttons to gripper commands.

Subscribes to the same 'joy' topic as ugv_tools/joy_ctrl.py (multiple nodes
can subscribe to one topic), so this runs alongside the existing drive
teleop without touching it. Publishes plain-string commands on 'gripper_cmd'
(consumed by ugv_gripper/gripper_node.py).

Button indices are hardware-specific (depend on the controller/driver) and
MUST be calibrated for your controller: run `ros2 topic echo /joy` while
pressing each button/axis and fill in the indices below via parameters,
e.g. in a launch file or with --ros-args -p btn_up:=3

Any command parameter left at -1 is disabled (no accidental triggers).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class GripperJoyCtrl(Node):
    def __init__(self):
        super().__init__('gripper_joy_ctrl')

        # ---- button index parameters (default: disabled, calibrate per controller) ----
        self.declare_parameter('btn_up', -1)
        self.declare_parameter('btn_down', -1)
        self.declare_parameter('btn_in', -1)
        self.declare_parameter('btn_out', -1)
        self.declare_parameter('btn_push', -1)
        self.declare_parameter('btn_pull', -1)
        self.declare_parameter('btn_stop', -1)

        self.button_command = {
            self.get_parameter('btn_up').value: 'up',
            self.get_parameter('btn_down').value: 'down',
            self.get_parameter('btn_in').value: 'in',
            self.get_parameter('btn_out').value: 'out',
            self.get_parameter('btn_push').value: 'push',
            self.get_parameter('btn_pull').value: 'pull',
            self.get_parameter('btn_stop').value: 'stop',
        }
        self.button_command.pop(-1, None)

        if not self.button_command:
            self.get_logger().warn(
                'No gripper buttons configured (all indices are -1). '
                'Run "ros2 topic echo /joy" and set btn_* parameters to calibrate.'
            )

        self.cmd_pub = self.create_publisher(String, 'gripper_cmd', 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.prev_buttons = []

    def joy_callback(self, msg: Joy):
        buttons = msg.buttons
        if not self.prev_buttons:
            self.prev_buttons = [0] * len(buttons)

        for index, command in self.button_command.items():
            if index < 0 or index >= len(buttons):
                continue
            pressed_now = buttons[index] == 1
            pressed_before = self.prev_buttons[index] == 1 if index < len(self.prev_buttons) else False
            if pressed_now and not pressed_before:
                self.cmd_pub.publish(String(data=command))

        self.prev_buttons = list(buttons)


def main(args=None):
    rclpy.init(args=args)
    node = GripperJoyCtrl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
