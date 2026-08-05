#!/usr/bin/env python3
"""Gripper arm control node - direct Jetson GPIO version (no PCA9685/I2C).

Port of the "Smart Servo Controller V5" Arduino reference sketch
(jetson_trial_2.ino), adapted to run natively on the Jetson via
Jetson.GPIO software PWM instead of an I2C PCA9685 driver board (the
PCA9685 board used previously is suspected dead/damaged).

Hardware (physical/BOARD pin numbers):
  - Roller (continuous-rotation) servo, in/out          -> pin 32
  - Left arm servo (180 degree)                         -> pin 24
  - Right arm servo (180 degree, mirrored)               -> pin 26
  - 2 GPIO outputs driving the relay-controlled linear
    actuator (DC motor)                                  -> pins 29 / 31
  - 1 GPIO input for the homing microswitch              -> pin 33

Commands accepted as plain strings on the 'gripper_cmd' topic:
  in, out, stop, home, up, down, push, pull, mstop, status
  rollspeed <0-100>, inspeed <0-100>, outspeed <0-100>, armspeed <0-100>

Status/log messages are published on 'gripper_state'.

NOTE: unlike the previous PCA9685-based implementation, the relay-driven
linear actuator (push/pull) has NO automatic timeout here - it matches
the V5 reference sketch exactly, which relies on 'mstop'/'stop' (or a
physical end-stop) to halt it. Flag this if a timeout safety net is
still wanted.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import Jetson.GPIO as GPIO

# =====================================================
# ROLLER (continuous rotation servo) CALIBRATION
# Only these three values determine direction.
# =====================================================
ROLLER_STOP_US = 1500
ROLLER_FULL_IN_US = 1000
ROLLER_FULL_OUT_US = 2000

# =====================================================
# ARM (mirrored 180 degree servos) CALIBRATION
# =====================================================
ARM_UP_US = 500
ARM_DOWN_US = 2500
MIRROR_CENTER = 3000

# =====================================================
# HOMING
# =====================================================
HOMING_TIME = 0.150       # seconds
SWITCH_DEBOUNCE = 0.025   # seconds

# =====================================================
# ARM SPEED (movement duration, seconds)
# =====================================================
ARM_FASTEST = 0.030
ARM_SLOWEST = 0.800

# =====================================================
# PWM
# =====================================================
PWM_FREQ_HZ = 50
PWM_PERIOD_US = 1_000_000.0 / PWM_FREQ_HZ  # 20000us at 50Hz

TICK_PERIOD = 0.02  # 50 Hz, same cadence as the Arduino loop()

# =====================================================
# STATES
# =====================================================
ROLLER_STOPPED = 'stopped'
ROLLER_IN = 'in'
ROLLER_OUT = 'out'
ROLLER_HOMING = 'homing'

RELAY_STOPPED = 'stopped'
RELAY_PUSH = 'push'
RELAY_PULL = 'pull'

ARM_STATE_UP = 'up'
ARM_STATE_DOWN = 'down'


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def remap(value, in_min, in_max, out_min, out_max):
    return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min)


class ServoPWM:
    """Thin wrapper around Jetson.GPIO software PWM, taking pulse widths in
    microseconds (matching the Arduino Servo library's writeMicroseconds())
    instead of a raw duty-cycle percentage."""

    def __init__(self, pin):
        self.pin = pin
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, PWM_FREQ_HZ)
        self._pwm.start(0)

    def write_microseconds(self, pulse_us):
        duty = clamp(pulse_us / PWM_PERIOD_US * 100.0, 0.0, 100.0)
        self._pwm.ChangeDutyCycle(duty)

    def stop(self):
        self._pwm.stop()


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Relay polarity empirically confirmed on this hardware (NOT the
        # same as the Arduino reference sketch's LOW/HIGH convention, which
        # was never tested on this specific wiring).
        self.RELAY_ON = GPIO.HIGH
        self.RELAY_OFF = GPIO.LOW

        # ---- parameters (hardware wiring) ----
        self.declare_parameter('roller_pin', 32)
        self.declare_parameter('left_pin', 24)
        self.declare_parameter('right_pin', 26)
        self.declare_parameter('relay1_pin', 29)
        self.declare_parameter('relay2_pin', 31)
        self.declare_parameter('home_switch_pin', 33)

        self.roller_pin = self.get_parameter('roller_pin').value
        self.left_pin = self.get_parameter('left_pin').value
        self.right_pin = self.get_parameter('right_pin').value
        self.relay1_pin = self.get_parameter('relay1_pin').value
        self.relay2_pin = self.get_parameter('relay2_pin').value
        self.home_switch_pin = self.get_parameter('home_switch_pin').value

        # ---- GPIO ----
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.relay1_pin, GPIO.OUT, initial=self.RELAY_OFF)
        GPIO.setup(self.relay2_pin, GPIO.OUT, initial=self.RELAY_OFF)
        # NOTE: Jetson.GPIO ignores pull_up_down on this platform/carrier
        # board (confirmed via a UserWarning during earlier testing) - an
        # external pull-up resistor (e.g. 10k to 3.3V) on home_switch_pin
        # is required for a reliable, non-floating reading.
        GPIO.setup(self.home_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.roller_servo = ServoPWM(self.roller_pin)
        self.left_servo = ServoPWM(self.left_pin)
        self.right_servo = ServoPWM(self.right_pin)

        # ---- roller state ----
        self.roller_state = ROLLER_STOPPED
        self.roller_in_speed = 100
        self.roller_out_speed = 100
        self.homing_start = 0.0

        # ---- arm state ----
        self.arm_state = ARM_STATE_UP
        self.arm_speed = 100
        self.arm_start_pulse = ARM_UP_US
        self.arm_target_pulse = ARM_UP_US
        self.arm_current_pulse = ARM_UP_US
        self.arm_move_start = 0.0
        self.arm_move_time = ARM_FASTEST

        # ---- relay state ----
        self.relay_state = RELAY_STOPPED

        # ---- homing switch debounce (edge-detect via polling) ----
        self.prev_switch_pressed = False
        self.last_switch_time = 0.0

        # ---- ROS interface ----
        self.create_subscription(String, 'gripper_cmd', self.cmd_callback, 10)
        self.state_pub = self.create_publisher(String, 'gripper_state', 10)

        self.roller_servo.write_microseconds(ROLLER_STOP_US)
        self.left_servo.write_microseconds(ARM_UP_US)
        self.right_servo.write_microseconds(MIRROR_CENTER - ARM_UP_US)
        self.update_relay()

        self.create_timer(TICK_PERIOD, self.update)

        self.log('Smart Servo Controller V5 Ready (Jetson native)')

    # =====================================================
    # Helpers
    # =====================================================
    def log(self, msg: str):
        self.get_logger().info(msg)
        self.state_pub.publish(String(data=msg))

    # =====================================================
    # Roller (continuous rotation servo)
    # =====================================================
    def calculate_roller_pulse(self, inward, speed):
        speed = clamp(speed, 0, 100)
        if inward:
            return remap(speed, 0, 100, ROLLER_STOP_US, ROLLER_FULL_IN_US)
        return remap(speed, 0, 100, ROLLER_STOP_US, ROLLER_FULL_OUT_US)

    def set_roller_state(self, state):
        self.roller_state = state
        if state == ROLLER_HOMING:
            self.homing_start = time.monotonic()
        self.update_roller()

    def update_roller(self):
        if self.roller_state == ROLLER_STOPPED:
            pulse = ROLLER_STOP_US
        elif self.roller_state == ROLLER_IN:
            pulse = self.calculate_roller_pulse(True, self.roller_in_speed)
        elif self.roller_state == ROLLER_OUT:
            pulse = self.calculate_roller_pulse(False, self.roller_out_speed)
        elif self.roller_state == ROLLER_HOMING:
            pulse = ROLLER_FULL_IN_US
        else:
            pulse = ROLLER_STOP_US
        self.roller_servo.write_microseconds(pulse)

    # =====================================================
    # Arm (mirrored 180 degree servos)
    # =====================================================
    def calculate_arm_duration(self):
        speed = clamp(self.arm_speed, 0, 100)
        return remap(speed, 0, 100, ARM_SLOWEST, ARM_FASTEST)

    def move_arm(self, up):
        self.arm_start_pulse = self.arm_current_pulse
        if up:
            self.arm_target_pulse = ARM_UP_US
            self.arm_state = ARM_STATE_UP
        else:
            self.arm_target_pulse = ARM_DOWN_US
            self.arm_state = ARM_STATE_DOWN
        self.arm_move_start = time.monotonic()
        self.arm_move_time = self.calculate_arm_duration()

    def update_arm(self):
        if self.arm_current_pulse == self.arm_target_pulse:
            return

        elapsed = time.monotonic() - self.arm_move_start
        if elapsed >= self.arm_move_time:
            self.arm_current_pulse = self.arm_target_pulse
        else:
            progress = elapsed / self.arm_move_time
            self.arm_current_pulse = (
                self.arm_start_pulse
                + (self.arm_target_pulse - self.arm_start_pulse) * progress
            )

        self.left_servo.write_microseconds(self.arm_current_pulse)
        self.right_servo.write_microseconds(MIRROR_CENTER - self.arm_current_pulse)

    # =====================================================
    # Homing (switch polled at 50Hz instead of a hardware interrupt -
    # equivalent behavior, since the Arduino ISR only ever set a flag
    # that was checked once per loop() iteration anyway)
    # =====================================================
    def check_home_switch(self):
        pressed = GPIO.input(self.home_switch_pin) == GPIO.LOW

        if pressed and not self.prev_switch_pressed:
            now = time.monotonic()
            if now - self.last_switch_time >= SWITCH_DEBOUNCE:
                self.last_switch_time = now
                # Homing only interrupts OUT movement
                if self.roller_state == ROLLER_OUT:
                    self.set_roller_state(ROLLER_HOMING)

        self.prev_switch_pressed = pressed

    def update_homing(self):
        if self.roller_state != ROLLER_HOMING:
            return

        self.roller_servo.write_microseconds(ROLLER_FULL_IN_US)

        if time.monotonic() - self.homing_start >= HOMING_TIME:
            self.set_roller_state(ROLLER_STOPPED)
            self.log('Homing complete.')

    # =====================================================
    # Relay-driven linear actuator
    # =====================================================
    def update_relay(self):
        if self.relay_state == RELAY_STOPPED:
            GPIO.output(self.relay1_pin, self.RELAY_OFF)
            GPIO.output(self.relay2_pin, self.RELAY_OFF)
        elif self.relay_state == RELAY_PUSH:
            GPIO.output(self.relay1_pin, self.RELAY_ON)
            GPIO.output(self.relay2_pin, self.RELAY_OFF)
        elif self.relay_state == RELAY_PULL:
            GPIO.output(self.relay1_pin, self.RELAY_OFF)
            GPIO.output(self.relay2_pin, self.RELAY_ON)

    def relay_push(self):
        self.relay_state = RELAY_PUSH
        self.update_relay()

    def relay_pull(self):
        self.relay_state = RELAY_PULL
        self.update_relay()

    def relay_stop(self):
        self.relay_state = RELAY_STOPPED
        self.update_relay()

    # =====================================================
    # Emergency stop
    # =====================================================
    def emergency_stop(self):
        self.set_roller_state(ROLLER_STOPPED)
        self.relay_stop()

        # Leave the arm exactly where it is (cancel any in-progress move)
        self.arm_start_pulse = self.arm_current_pulse
        self.arm_target_pulse = self.arm_current_pulse

        self.log('EMERGENCY STOP')

    # =====================================================
    # Status
    # =====================================================
    def print_status(self):
        switch_state = 'PRESSED' if GPIO.input(self.home_switch_pin) == GPIO.LOW else 'RELEASED'
        lines = [
            '========== STATUS ==========',
            f'Roller State      : {self.roller_state.upper()}',
            f'Arm Position      : {self.arm_state.upper()}',
            f'Arm Pulse (us)    : {self.arm_current_pulse:.0f}',
            f'IN Speed (%)      : {self.roller_in_speed}',
            f'OUT Speed (%)     : {self.roller_out_speed}',
            f'Arm Speed (%)     : {self.arm_speed}',
            f'Microswitch       : {switch_state}',
            f'Relay State       : {self.relay_state.upper()}',
            f'Homing Active     : {"YES" if self.roller_state == ROLLER_HOMING else "NO"}',
            '============================',
        ]
        for line in lines:
            self.log(line)

    # =====================================================
    # Main tick (replaces Arduino loop())
    # =====================================================
    def update(self):
        self.check_home_switch()
        self.update_roller()
        self.update_arm()
        self.update_homing()
        self.update_relay()

    # =====================================================
    # Command handling (mirrors processCommand())
    # =====================================================
    def cmd_callback(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return

        if command == 'in':
            self.set_roller_state(ROLLER_IN)
            return
        if command == 'out':
            self.set_roller_state(ROLLER_OUT)
            return
        if command == 'stop':
            self.emergency_stop()
            return
        if command == 'home':
            self.set_roller_state(ROLLER_HOMING)
            return
        if command == 'up':
            self.move_arm(True)
            return
        if command == 'down':
            self.move_arm(False)
            return
        if command == 'push':
            self.relay_push()
            return
        if command == 'pull':
            self.relay_pull()
            return
        if command == 'mstop':
            self.relay_stop()
            return
        if command == 'status':
            self.print_status()
            return

        for prefix, setter in (
            ('rollspeed', self._set_roll_speed),
            ('inspeed', self._set_in_speed),
            ('outspeed', self._set_out_speed),
            ('armspeed', self._set_arm_speed),
        ):
            if command.startswith(prefix):
                value_str = command[len(prefix):].strip()
                if not value_str:
                    self.log('Missing value.')
                    return
                try:
                    value = int(value_str)
                except ValueError:
                    self.log('Speed must be 0-100.')
                    return
                if not 0 <= value <= 100:
                    self.log('Speed must be 0-100.')
                    return
                setter(value)
                return

        self.log(f'Unknown command: {command}')

    def _set_roll_speed(self, value):
        self.roller_in_speed = value
        self.roller_out_speed = value
        self.log('Roller speed updated.')

    def _set_in_speed(self, value):
        self.roller_in_speed = value
        self.log('IN speed updated.')

    def _set_out_speed(self, value):
        self.roller_out_speed = value
        self.log('OUT speed updated.')

    def _set_arm_speed(self, value):
        self.arm_speed = value
        self.log('Arm speed updated.')

    def destroy_node(self):
        self.roller_servo.write_microseconds(ROLLER_STOP_US)
        self.relay_stop()
        self.roller_servo.stop()
        self.left_servo.stop()
        self.right_servo.stop()
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
