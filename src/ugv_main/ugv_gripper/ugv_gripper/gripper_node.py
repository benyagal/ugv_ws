#!/usr/bin/env python3
"""Gripper arm control node.

Direct 1:1 (non-blocking) port of the Arduino sketch
(All_motions_preparation_for_jetson.ino) to run natively on the Jetson,
talking to the hardware over:
  - I2C (PCA9685 16-channel PWM driver, 3 channels used) -> physical pins
    27 (SDA) / 28 (SCL)
  - 2 GPIO outputs driving the relay-controlled linear actuator (DC motor)
    -> physical pins 29 (GPIO01) and 31 (GPIO11)
  - 1 GPIO input for the homing microswitch -> physical pin 33 (GPIO13)

Commands are accepted as plain strings on the 'gripper_cmd' topic, using the
exact same vocabulary as the Arduino serial protocol:
  in, out, up, down, push, pull, stop

Status/log messages are published on 'gripper_state' (mirrors Serial.println).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import Jetson.GPIO as GPIO

from ugv_gripper.pca9685 import PCA9685

# =====================================================
# PCA9685 CHANNELS
# =====================================================
SERVO1_CHANNEL = 0   # Continuous rotation servo (in/out + homing)
SERVO2_CHANNEL = 1   # 180 degree servo (up/down)
SERVO3_CHANNEL = 2   # 180 degree servo, mirrored counterpart of SERVO2

# =====================================================
# SERVO PWM VALUES
# =====================================================
SERVO1_STOP_PWM = 320
SERVO1_MAX_CW_PWM = 380
SERVO1_MAX_CCW_PWM = 260

SERVO_MIN_PWM = 120
SERVO_MAX_PWM = 480

# =====================================================
# TIMES (seconds)
# =====================================================
IN_TIME = 20.0
MOTOR_TIME = 18.0
HOME_BACK_TIME = 0.2
HOME_DEBOUNCE_TIME = 0.02

TICK_PERIOD = 0.02  # 50 Hz, same cadence as the Arduino loop()

# =====================================================
# STATES
# =====================================================
SERVO1_IDLE = 'idle'
SERVO1_IN = 'in'
SERVO1_OUT = 'out'
SERVO1_BACKOFF = 'backoff'

MOTOR_IDLE = 'idle'
MOTOR_PUSH = 'push'
MOTOR_PULL = 'pull'


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Relays are active-low, matching the Arduino RELAY_ON/RELAY_OFF definitions
        self.RELAY_ON = GPIO.LOW
        self.RELAY_OFF = GPIO.HIGH

        # ---- parameters (hardware wiring) ----
        self.declare_parameter('i2c_bus', 7)
        self.declare_parameter('pca9685_address', 0x42)
        self.declare_parameter('relay1_pin', 29)
        self.declare_parameter('relay2_pin', 31)
        self.declare_parameter('home_switch_pin', 33)

        i2c_bus = self.get_parameter('i2c_bus').value
        pca_addr = self.get_parameter('pca9685_address').value
        self.relay1_pin = self.get_parameter('relay1_pin').value
        self.relay2_pin = self.get_parameter('relay2_pin').value
        self.home_switch_pin = self.get_parameter('home_switch_pin').value

        # ---- I2C / PCA9685 ----
        self.pwm = PCA9685(i2c_bus, pca_addr)
        self.pwm.set_pwm_freq(50)

        # ---- GPIO ----
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.relay1_pin, GPIO.OUT, initial=self.RELAY_OFF)
        GPIO.setup(self.relay2_pin, GPIO.OUT, initial=self.RELAY_OFF)
        GPIO.setup(self.home_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # ---- state (mirrors the Arduino globals) ----
        self.servo1_state = SERVO1_IDLE
        self.servo1_timer = 0.0
        self.backoff_timer = 0.0
        self.home_seen = False
        self.home_debounce_timer = 0.0

        self.motor_state = MOTOR_IDLE
        self.motor_timer = 0.0

        # ---- ROS interface ----
        self.create_subscription(String, 'gripper_cmd', self.cmd_callback, 10)
        self.state_pub = self.create_publisher(String, 'gripper_state', 10)

        self.servo1_stop()
        self.motor_stop()

        self.pwm.set_pwm(SERVO2_CHANNEL, 0, SERVO_MIN_PWM)
        self.pwm.set_pwm(SERVO3_CHANNEL, 0, SERVO_MAX_PWM)

        self.create_timer(TICK_PERIOD, self.update)

        self.log('SYSTEM READY')

    # =====================================================
    # Helpers
    # =====================================================
    def log(self, msg: str):
        self.get_logger().info(msg)
        self.state_pub.publish(String(data=msg))

    def is_busy(self):
        return self.servo1_state != SERVO1_IDLE or self.motor_state != MOTOR_IDLE

    # =====================================================
    # Continuous servo (servo1)
    # =====================================================
    def servo1_cw(self):
        self.pwm.set_pwm(SERVO1_CHANNEL, 0, SERVO1_MAX_CW_PWM)
        self.log('Servo1 CW')

    def servo1_ccw(self):
        self.pwm.set_pwm(SERVO1_CHANNEL, 0, SERVO1_MAX_CCW_PWM)
        self.log('Servo1 CCW')

    def servo1_stop(self):
        self.pwm.set_pwm(SERVO1_CHANNEL, 0, SERVO1_STOP_PWM)
        self.log('Servo1 Stop')

    # =====================================================
    # Relay-driven linear actuator
    # =====================================================
    def motor_push(self):
        GPIO.output(self.relay1_pin, self.RELAY_ON)
        GPIO.output(self.relay2_pin, self.RELAY_ON)
        self.log('Motor PUSH')

    def motor_pull(self):
        GPIO.output(self.relay1_pin, self.RELAY_OFF)
        GPIO.output(self.relay2_pin, self.RELAY_OFF)
        self.log('Motor PULL')

    def motor_stop(self):
        GPIO.output(self.relay1_pin, self.RELAY_OFF)
        GPIO.output(self.relay2_pin, self.RELAY_ON)
        self.log('Motor STOP')

    # =====================================================
    # Commands (mirrors the Arduino command_* functions)
    # =====================================================
    def command_in(self):
        self.log('IN')
        self.servo1_state = SERVO1_IN
        self.servo1_timer = 0.0
        self.servo1_ccw()

    def command_out(self):
        self.log('OUT')
        self.home_seen = False
        self.servo1_state = SERVO1_OUT
        self.servo1_cw()

    def command_up(self):
        self.log('UP')
        self.pwm.set_pwm(SERVO2_CHANNEL, 0, SERVO_MAX_PWM)
        self.pwm.set_pwm(SERVO3_CHANNEL, 0, SERVO_MIN_PWM)
        self.log('UP complete')

    def command_down(self):
        self.log('DOWN')
        self.pwm.set_pwm(SERVO2_CHANNEL, 0, SERVO_MIN_PWM)
        self.pwm.set_pwm(SERVO3_CHANNEL, 0, SERVO_MAX_PWM)
        self.log('DOWN complete')

    def command_push(self):
        self.log('PUSH')
        self.motor_state = MOTOR_PUSH
        self.motor_timer = 0.0
        self.motor_push()

    def command_pull(self):
        self.log('PULL')
        self.motor_state = MOTOR_PULL
        self.motor_timer = 0.0
        self.motor_pull()

    def stop_everything(self):
        self.log('STOP')

        self.servo1_stop()
        self.servo1_state = SERVO1_IDLE
        self.home_seen = False

        self.motor_stop()
        self.motor_state = MOTOR_IDLE

        self.servo1_timer = 0.0
        self.motor_timer = 0.0
        self.backoff_timer = 0.0
        self.home_debounce_timer = 0.0

    # =====================================================
    # Main tick (replaces Arduino loop())
    # =====================================================
    def update(self):
        try:
            self.update_continuous_servo()
            self.update_dc_motor()
        except OSError as e:
            self.handle_i2c_failure(e)

    def handle_i2c_failure(self, error: Exception):
        # A persistent I2C bus fault (e.g. repeated "arbitration lost") should
        # never be allowed to crash the whole node - that leaves the motor/
        # relay in whatever state they were and requires a manual restart.
        # Instead, log it and force everything to a safe stopped state; the
        # next command will retry the I2C bus from a clean slate.
        self.get_logger().error(f'I2C bus failure, stopping gripper: {error}')
        self.servo1_state = SERVO1_IDLE
        self.home_seen = False
        self.motor_state = MOTOR_IDLE
        try:
            self.motor_stop()
        except OSError:
            pass
        self.log('I2C ERROR - STOPPED')

    def update_continuous_servo(self):
        if self.servo1_state == SERVO1_IDLE:
            return

        if self.servo1_state == SERVO1_IN:
            self.servo1_timer += TICK_PERIOD
            if self.servo1_timer >= IN_TIME:
                self.servo1_stop()
                self.servo1_state = SERVO1_IDLE
                self.log('IN complete')
            return

        if self.servo1_state == SERVO1_OUT:
            # INPUT_PULLUP, trigger when HIGH (same as the Arduino sketch)
            if GPIO.input(self.home_switch_pin) == GPIO.HIGH:
                if not self.home_seen:
                    self.home_seen = True
                    self.home_debounce_timer = 0.0
                else:
                    self.home_debounce_timer += TICK_PERIOD

                if self.home_debounce_timer >= HOME_DEBOUNCE_TIME:
                    self.log('Home detected')
                    self.servo1_ccw()
                    self.backoff_timer = 0.0
                    self.servo1_state = SERVO1_BACKOFF
                    self.home_seen = False
            else:
                self.home_seen = False
            return

        if self.servo1_state == SERVO1_BACKOFF:
            self.backoff_timer += TICK_PERIOD
            if self.backoff_timer >= HOME_BACK_TIME:
                self.servo1_stop()
                self.servo1_state = SERVO1_IDLE
                self.log('Home complete')
            return

    def update_dc_motor(self):
        if self.motor_state == MOTOR_IDLE:
            return

        self.motor_timer += TICK_PERIOD
        if self.motor_timer >= MOTOR_TIME:
            self.motor_stop()
            if self.motor_state == MOTOR_PUSH:
                self.log('PUSH complete')
            else:
                self.log('PULL complete')
            self.motor_state = MOTOR_IDLE

    # =====================================================
    # Command handling (mirrors readSerial())
    # =====================================================
    def cmd_callback(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return

        if command == 'stop':
            self.stop_everything()
            return

        if self.is_busy():
            self.log('System Busy')
            return

        try:
            if command == 'in':
                self.command_in()
            elif command == 'out':
                self.command_out()
            elif command == 'up':
                self.command_up()
            elif command == 'down':
                self.command_down()
            elif command == 'push':
                self.command_push()
            elif command == 'pull':
                self.command_pull()
            else:
                self.log(f'Unknown command: {command}')
        except OSError as e:
            self.handle_i2c_failure(e)

    def destroy_node(self):
        self.servo1_stop()
        self.motor_stop()
        self.pwm.close()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
