#!/usr/bin/env python3
"""Gripper arm control node - PCA9685 (I2C) servos + native Jetson GPIO for
the relay-driven linear actuator and homing switch.

Port of the "Smart Servo Controller V5" Arduino reference sketch
(jetson_trial_2.ino). Servos moved BACK to the PCA9685 I2C PWM driver board
(this project's original hardware) after the Jetson-native bit-banged PWM
version (a ServoPWM class previously in this file) turned out to cause the
"electronics going wild" symptom it was meant to work around: Jetson.GPIO's
GPIO.output() is not fast/precise enough for microsecond-accurate PWM, and
running 3 concurrent bit-banging threads (roller + 2 arm servos, all with
non-zero holding pulses from startup) produced enough timing jitter and
electrical noise to also disturb the relay and homing switch lines. A
dedicated PWM IC generates all 3 servo signals in hardware, removing the
Jetson/Python timing from the loop entirely.

Hardware (physical/BOARD pin numbers):
  - PCA9685 I2C bus: wired to physical pins 27 (SDA) / 28 (SCL) - NOT the
    default/primary I2C bus (pins 3/5) that busio.I2C(board.SCL, board.SDA)
    would auto-select. Pins 27/28 are the secondary/ID-EEPROM-style I2C bus
    on the Jetson 40-pin header. Rather than relying on Adafruit Blinka's
    per-board pin-name lookup (which may not fully recognize this carrier
    board, same issue as Jetson.GPIO's own "not a Jetson Developer Kit"
    warning), the I2C bus is opened directly by /dev/i2c-N number - see the
    'i2c_bus' parameter below.
  - Roller (continuous-rotation) servo     -> PCA9685 channel 0
  - Left arm servo (180 degree)            -> PCA9685 channel 1
  - Right arm servo (180 degree, mirrored) -> PCA9685 channel 2
  - 2 GPIO outputs driving the relay-controlled linear
    actuator (DC motor)                                  -> pins 29 / 31
  - 1 GPIO input for the homing microswitch              -> pin 33

Commands accepted as plain strings on the 'gripper_cmd' topic:
  in, out, stop, home, up, down, push, pull, mstop, status
  rollspeed <0-100>, inspeed <0-100>, outspeed <0-100>, armspeed <0-100>

Status/log messages are published on 'gripper_state'.

NOTE: the relay-driven linear actuator (push/pull) still has NO automatic
timeout - it relies on 'mstop'/'stop' (or a physical end-stop) to halt it.
Not addressed here; flag if a timeout safety net is still wanted.
"""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import Jetson.GPIO as GPIO
from adafruit_blinka.microcontroller.generic_linux.i2c import I2C as LinuxI2C
from adafruit_pca9685 import PCA9685

# =====================================================
# ROLLER (continuous rotation servo) CALIBRATION
# Only these three values determine direction.
# =====================================================
ROLLER_STOP_US = 1000
ROLLER_FULL_IN_US = 1500
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


class PCA9685Servo:
    """Thin wrapper around one PCA9685 channel, taking pulse widths in
    microseconds (matching the Arduino Servo library's writeMicroseconds()
    and the previous ServoPWM class's interface). All PWM timing happens in
    hardware on the PCA9685 chip - no Python/GIL involvement, unlike the
    bit-banged GPIO approach this replaces.
    """

    def __init__(self, pca, channel):
        self.pca = pca
        self.channel = channel

    def write_microseconds(self, pulse_us):
        pulse_us = clamp(pulse_us, 0.0, PWM_PERIOD_US)
        self.pca.channels[self.channel].duty_cycle = int(pulse_us / PWM_PERIOD_US * 0xFFFF)

    def stop(self):
        self.pca.channels[self.channel].duty_cycle = 0


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')

        # Relay polarity empirically confirmed on this hardware (NOT the
        # same as the Arduino reference sketch's LOW/HIGH convention, which
        # was never tested on this specific wiring).
        self.RELAY_ON = GPIO.HIGH
        self.RELAY_OFF = GPIO.LOW

        # ---- parameters (hardware wiring) ----
        self.declare_parameter('roller_channel', 0)
        self.declare_parameter('left_channel', 1)
        self.declare_parameter('right_channel', 2)
        self.declare_parameter('relay1_pin', 29)
        self.declare_parameter('relay2_pin', 31)
        self.declare_parameter('home_switch_pin', 33)
        # PCA9685 is wired to physical pins 27 (SDA) / 28 (SCL), which is a
        # different /dev/i2c-N bus than the Jetson's default/primary I2C bus
        # (pins 3/5) - adjust if it turns out to be a different bus number
        # on this carrier board (verify with `i2cdetect -y <bus>` showing
        # the device at i2c_address).
        self.declare_parameter('i2c_bus', 0)
        self.declare_parameter('i2c_address', 0x40)

        self.roller_channel = self.get_parameter('roller_channel').value
        self.left_channel = self.get_parameter('left_channel').value
        self.right_channel = self.get_parameter('right_channel').value
        self.relay1_pin = self.get_parameter('relay1_pin').value
        self.relay2_pin = self.get_parameter('relay2_pin').value
        self.home_switch_pin = self.get_parameter('home_switch_pin').value
        self.i2c_bus = self.get_parameter('i2c_bus').value
        self.i2c_address = self.get_parameter('i2c_address').value

        # ---- GPIO (relay + homing switch only - servos are on the PCA9685) ----
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.relay1_pin, GPIO.OUT, initial=self.RELAY_OFF)
        GPIO.setup(self.relay2_pin, GPIO.OUT, initial=self.RELAY_OFF)
        # NOTE: Jetson.GPIO ignores pull_up_down on this platform/carrier
        # board (confirmed via a UserWarning during earlier testing) - an
        # external pull-up resistor (e.g. 10k to 3.3V) on home_switch_pin
        # is required for a reliable, non-floating reading.
        GPIO.setup(self.home_switch_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # ---- PCA9685 (I2C, bus wired to pins 27/28 - see module docstring) ----
        self.get_logger().info(
            f'Connecting to PCA9685 at 0x{self.i2c_address:02X} on I2C bus '
            f'{self.i2c_bus} (pins 27/28)...'
        )
        i2c = LinuxI2C(self.i2c_bus)
        self.pca = PCA9685(i2c, address=self.i2c_address)
        self.pca.frequency = PWM_FREQ_HZ

        self.roller_servo = PCA9685Servo(self.pca, self.roller_channel)
        self.left_servo = PCA9685Servo(self.pca, self.left_channel)
        self.right_servo = PCA9685Servo(self.pca, self.right_channel)

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
        just_finished = elapsed >= self.arm_move_time
        if just_finished:
            self.arm_current_pulse = self.arm_target_pulse
        else:
            progress = elapsed / self.arm_move_time
            self.arm_current_pulse = (
                self.arm_start_pulse
                + (self.arm_target_pulse - self.arm_start_pulse) * progress
            )

        self.left_servo.write_microseconds(self.arm_current_pulse)
        self.right_servo.write_microseconds(MIRROR_CENTER - self.arm_current_pulse)

        if just_finished:
            self.log(f'{self.arm_state.upper()} complete')

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
            GPIO.output(self.relay2_pin, self.RELAY_ON)
        elif self.relay_state == RELAY_PUSH:
            GPIO.output(self.relay1_pin, self.RELAY_ON)
            GPIO.output(self.relay2_pin, self.RELAY_ON)
        elif self.relay_state == RELAY_PULL:
            GPIO.output(self.relay1_pin, self.RELAY_OFF)
            GPIO.output(self.relay2_pin, self.RELAY_OFF)

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

        # NOTE: the V5 reference sketch itself doesn't log anything for
        # these "happy path" commands (only STOP/homing-complete/speed
        # changes print) - added here purely for /gripper_state operator
        # feedback, matching the older node's behavior. Doesn't change any
        # actual motion logic.
        if command == 'in':
            self.log('IN')
            self.set_roller_state(ROLLER_IN)
            return
        if command == 'out':
            self.log('OUT')
            self.set_roller_state(ROLLER_OUT)
            return
        if command == 'stop':
            self.emergency_stop()
            return
        if command == 'home':
            self.log('HOME')
            self.set_roller_state(ROLLER_HOMING)
            return
        if command == 'up':
            self.log('UP')
            self.move_arm(True)
            return
        if command == 'down':
            self.log('DOWN')
            self.move_arm(False)
            return
        if command == 'push':
            self.log('PUSH')
            self.relay_push()
            return
        if command == 'pull':
            self.log('PULL')
            self.relay_pull()
            return
        if command == 'mstop':
            self.log('MSTOP')
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
        self.pca.deinit()
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
