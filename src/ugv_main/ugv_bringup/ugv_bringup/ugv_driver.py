#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32, Float32MultiArray
import subprocess
import time

from Rosmaster_Lib import Rosmaster

# Must match ugv_bringup.py's car_type/port - see
# /memories/repo/rosmaster_motor_controller.md for the investigation behind
# these values.
CAR_TYPE = 4
SERIAL_PORT = '/dev/ttyUSB0'

# Preliminary ground-measured calibration (2026-09-15): commanding 0.1 m/s for
# 2s actually moved the robot ~75-78cm (~0.38 m/s actual, ~3.8x too fast) -
# set_car_motion()'s real-world speed doesn't match the requested m/s 1:1 on
# this hardware. Dividing the commanded linear speed by this factor corrects
# for it. Only linear speed was measured this way - angular is NOT scaled
# (only its direction was confirmed correct, not its rate).
LINEAR_SPEED_CORRECTION_FACTOR = 3.8


class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.car = Rosmaster(car_type=CAR_TYPE, com=SERIAL_PORT)
        self.car.create_receive_threading()

        # Only log the "not implemented" servo/LED warnings once each,
        # instead of spamming on every message.
        self._warned_joint_states = False
        self._warned_led_ctrl = False

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)

        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)

        # Subscribe to voltage data (voltage topic)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

    # Callback for processing velocity commands
    def cmd_vel_callback(self, msg):
        # NOTE: physical direction was found reversed vs. cmd_vel intent on
        # the NEW board too (confirmed empirically 2026-09-11: commanding
        # +0.2 m/s forward drove the robot backward, all 4 wheels attached) -
        # inverted here in software rather than rewiring the motors.
        linear_velocity = -msg.linear.x / LINEAR_SPEED_CORRECTION_FACTOR
        angular_velocity = -msg.angular.z


        # Apply minimum threshold to angular velocity if linear velocity is zero
        if linear_velocity == 0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2

        # set_car_motion takes real m/s / rad/s directly (confirmed from
        # Yahboom's own reference driver) - no scaling needed. vy is always
        # 0 since CAR_FOURWHEEL ignores it (pure differential drive).
        self.car.set_car_motion(linear_velocity, 0.0, angular_velocity)

    # Callback for processing joint state updates
    def joint_states_callback(self, msg):
        # TODO: pan-tilt camera servo control not yet implemented for the new
        # board. Old board used a JSON T:134 command; Rosmaster_Lib's API is
        # different (set_pwm_servo/set_uart_servo_angle), and it's unconfirmed
        # whether the pan-tilt servo is even wired to the new board.
        if not self._warned_joint_states:
            self.get_logger().warn(
                "joint_states_callback: pan-tilt servo control not yet implemented for the new Rosmaster board"
            )
            self._warned_joint_states = True

    # Callback for processing LED control commands
    def led_ctrl_callback(self, msg):
        # TODO: same as above - LED wiring/API unconfirmed for the new board.
        if not self._warned_led_ctrl:
            self.get_logger().warn(
                "led_ctrl_callback: LED control not yet implemented for the new Rosmaster board"
            )
            self._warned_led_ctrl = True

    # Callback for processing voltage data
    def voltage_callback(self, msg):
        voltage_value = msg.data

        # If voltage drops below a threshold, play a low battery warning sound
        if 0.1 < voltage_value < 9: 
            subprocess.run(['aplay', '-D', 'plughw:3,0', '/home/ws/ugv_ws/src/ugv_main/ugv_bringup/ugv_bringup/low_battery.wav'])
            time.sleep(5)

def main(args=None):
    rclpy.init(args=args)
    node = UgvDriver("ugv_driver")
    
    try:
        rclpy.spin(node)  # Keep the node running and handling callbacks
    except KeyboardInterrupt:
        pass  # Graceful shutdown on user interrupt
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
