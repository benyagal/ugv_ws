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

# CUSTOM CLOSED-LOOP SPEED CONTROL (2026-09-15) - replaces set_car_motion().
# The firmware's own speed PID was found to be fundamentally broken for our
# motors: any commanded speed from ~0.06 to 2.0 m/s saturates to the SAME
# fixed real speed, and retuning its gains via set_pid_param() only produced
# either no motion (weak gain) or violent oscillation (stronger gain) - its
# encoder-feedback scaling uses wheel/encoder constants hardcoded for
# Yahboom's own motor, which don't match ours. Full investigation and all
# calibration data in /memories/repo/rosmaster_motor_controller.md.
#
# Raw PWM (set_motor(), which bypasses the firmware's encoder/PID loop
# entirely) instead gives a smooth, monotonic, stable real-speed response for
# both linear and angular motion - so we drive the motors with set_motor()
# and close our own feedforward+PI speed loop here in Python, using
# get_motion_data() (still computed by the firmware from real encoder counts,
# just with a roughly constant scale error) as feedback.
CONTROL_PERIOD = 0.1  # seconds (10 Hz) - matches the rate validated during calibration
CMD_VEL_TIMEOUT = 0.5  # seconds - stop the motors if no cmd_vel arrives within this
# Drop to open-loop feedforward if ugv_bringup stops publishing motion_raw:
# holding the last (stale) measurement would wind the PI loops up against a
# frozen error, but cutting the motors dead would also make a transient
# publisher stall on the Jetson look like an emergency stop mid-drive.
MOTION_FEEDBACK_TIMEOUT = 0.5  # seconds
# Same idea, for the per-side wheel speed used by REVERSE_BREAKAWAY_BOOST
# below - published on the same 0.04s timer as motion_raw, so this should
# normally never trip while ugv_bringup is alive.
ODOM_FEEDBACK_TIMEOUT = 0.5  # seconds

# get_motion_data() overreports real speed by a roughly constant factor,
# found from repeated real-distance/rotation measurements across a wide
# speed range (see memory file). Linear and angular have DIFFERENT factors
# since the firmware computes them from different, separately-wrong
# hardcoded constants (wheel circumference/pulses-per-rev vs. wheelbase).
LINEAR_TELEMETRY_CORRECTION = 0.82
ANGULAR_TELEMETRY_CORRECTION = 0.65

# Feedforward: duty (0-100) needed for a given |target speed|, fitted from
# real-world calibration runs (tune_motor_pid.py --raw / --raw --spin,
# 2026-09-15 - see memory file for the raw data points behind this fit).
LINEAR_FF_SLOPE = 107.2   # duty per m/s
LINEAR_FF_OFFSET = 18.6   # duty needed to overcome stiction/deadband
ANGULAR_FF_SLOPE = 25.8   # duty per rad/s
ANGULAR_FF_OFFSET = 29.9  # duty needed to overcome stiction/deadband

# Extra duty applied ONLY while an axis is still stalled. The front-right
# wheel needs ~40-45 duty to break static friction (2026-09-15 test), more
# than the fitted offset alone gives at low targets. This used to be folded
# into ANGULAR_FF_OFFSET (raised 29.9 -> 38.0), but that is a PERMANENT +8.1
# duty (~+0.31 rad/s real) bias on every nonzero angular command: the PI trim
# needs ~2s of uninterrupted error to integrate it away, so long calibration
# spins looked fine while short commands (Nav2 heading corrections, teleop
# taps) over-rotated by ~0.3 rad/s every single time. Applying it as a kick
# that disappears as soon as the axis actually moves keeps the stiction fix
# without the steady-state bias.
ANGULAR_BREAKAWAY_BOOST = 8.1    # duty
ANGULAR_BREAKAWAY_SPEED = 0.05   # rad/s - below this the axis counts as stalled

# In-place turns were observed (2026-09-22) to stall on whichever side is
# commanded BACKWARD (positive duty, per the sign convention below) - the
# drivetrain needs more duty to break static friction in reverse than
# forward. ANGULAR_BREAKAWAY_BOOST above doesn't cover this: it's gated on
# the combined measured_angular, which already reads as "moving" once the
# forward side alone starts spinning and pivots the chassis around the
# still-stalled reverse side.
#
# First attempt keyed this off each side's own COMMANDED duty magnitude
# (boost while duty < 55), but at the turn speeds this robot actually uses
# the commanded duty is already ~50-57 from the feedforward alone - always
# past the threshold, so that version never fired. Replaced with genuine
# per-side feedback: ugv_bringup's odom_raw carries real per-wheel-side
# distance, differentiated here into real per-side speed, so the stalled
# side can be detected directly instead of guessed from duty.
PER_SIDE_REVERSE_BREAKAWAY_BOOST = 20.0  # extra duty for a stalled backward-commanded side
PER_SIDE_BREAKAWAY_SPEED = 0.02          # m/s - below this a side counts as stalled

# PI trim gains (correct the feedforward's residual error) - kept modest
# since this loop runs in plain Python at CONTROL_PERIOD, not in firmware.
LINEAR_KP = 40.0
LINEAR_KI = 20.0
ANGULAR_KP = 15.0
ANGULAR_KI = 15.0  # raised from 8.0 to correct residual undershoot faster

DUTY_LIMIT = 100.0


class SpeedPIController:
    """Feedforward + PI trim controller for one motion axis (linear or
    angular), producing a signed duty value in [-DUTY_LIMIT, DUTY_LIMIT].

    The feedforward term does most of the work (it's a fit of duty vs. real
    measured speed from real hardware tests), the PI term only trims the
    residual error - this keeps the loop well-behaved without needing large,
    windup-prone gains.
    """

    def __init__(self, ff_slope, ff_offset, kp, ki, breakaway_boost=0.0, breakaway_speed=0.0):
        self.ff_slope = ff_slope
        self.ff_offset = ff_offset
        self.kp = kp
        self.ki = ki
        self.breakaway_boost = breakaway_boost
        self.breakaway_speed = breakaway_speed
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, measured, dt, closed_loop=True):
        if not closed_loop:
            # Pure feedforward: it is a fit of real measured hardware data, so
            # the robot still tracks the commanded speed reasonably well - it
            # just loses the residual trim.
            self.reset()
            measured = target

        if target == 0.0:
            # No feedforward (its stiction offset has no meaningful sign at
            # zero target), but the PI trim below KEEPS RUNNING so the axis is
            # actively held at zero. This is what makes a straight drive stay
            # straight despite the chassis' mechanical left/right asymmetry -
            # returning 0 here instead left the angular loop open whenever
            # angular.z was 0, i.e. during every straight drive.
            feedforward = 0.0
        else:
            sign = 1.0 if target > 0 else -1.0
            feedforward = sign * (self.ff_slope * abs(target) + self.ff_offset)
            if self.breakaway_speed > 0.0 and abs(measured) < self.breakaway_speed:
                # Faded out linearly rather than switched off at the
                # threshold: a hard step made the axis creep just past
                # breakaway_speed, lose the boost, stall, regain it, and so
                # on - a stall/kick limit cycle that feels weak and never
                # completes a turn.
                stall = 1.0 - abs(measured) / self.breakaway_speed
                feedforward += sign * self.breakaway_boost * stall

        error = target - measured
        proposed_integral = self.integral + error * dt
        output = feedforward + self.kp * error + self.ki * proposed_integral

        # Anti-windup: only keep the integral update if it doesn't push the
        # output past the duty limit (clamped conditional integration).
        if -DUTY_LIMIT <= output <= DUTY_LIMIT:
            self.integral = proposed_integral
        return max(-DUTY_LIMIT, min(DUTY_LIMIT, output))


class UgvDriver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.car = Rosmaster(car_type=CAR_TYPE, com=SERIAL_PORT)
        # Deliberately NO create_receive_threading() here: this node is
        # write-only on the serial port. Rosmaster_Lib's receive thread reads
        # the tty one byte at a time, so running one here as well as in
        # ugv_bringup would split every incoming frame randomly between the
        # two processes and leave both with mostly checksum-failed packets.
        # Speed feedback comes from ugv_bringup's motion_raw topic instead.

        # Only log the "not implemented" servo/LED warnings once each,
        # instead of spamming on every message.
        self._warned_joint_states = False
        self._warned_led_ctrl = False

        # Custom closed-loop speed control state (see constants above).
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.measured_linear = 0.0
        self.measured_angular = 0.0
        self.last_cmd_vel_time = time.monotonic()
        self.last_motion_time = time.monotonic()
        self._last_control_time = time.monotonic()

        # Per-side real wheel speed (m/s), differentiated from odom_raw's
        # cumulative distance - see PER_SIDE_REVERSE_BREAKAWAY_BOOST above.
        self.measured_left_speed = 0.0
        self.measured_right_speed = 0.0
        self.last_odom_time = time.monotonic()
        self._last_odom_left_m = None
        self._last_odom_right_m = None

        self.linear_ctrl = SpeedPIController(LINEAR_FF_SLOPE, LINEAR_FF_OFFSET, LINEAR_KP, LINEAR_KI)
        self.angular_ctrl = SpeedPIController(
            ANGULAR_FF_SLOPE, ANGULAR_FF_OFFSET, ANGULAR_KP, ANGULAR_KI,
            breakaway_boost=ANGULAR_BREAKAWAY_BOOST,
            breakaway_speed=ANGULAR_BREAKAWAY_SPEED,
        )
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # Board speed telemetry, republished by ugv_bringup (sole serial reader)
        self.motion_sub_ = self.create_subscription(Twist, "motion_raw", self.motion_raw_callback, 10)

        # Per-side cumulative wheel distance, same source used for odometry -
        # differentiated below for the reverse-breakaway stall detection.
        self.odom_sub_ = self.create_subscription(Float32MultiArray, "odom/odom_raw", self.odom_raw_callback, 10)

        # Subscribe to joint states (ugv/joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'ugv/joint_states', self.joint_states_callback, 10)

        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 10)

        # Subscribe to voltage data (voltage topic)
        self.voltage_sub = self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)

    # Callback for processing velocity commands - just records the latest
    # target; the actual motor control happens in control_loop() below.
    def cmd_vel_callback(self, msg):
        # target_linear positive = forward (physical direction was found
        # reversed vs. cmd_vel intent, confirmed empirically 2026-09-11 - the
        # inversion is applied once, in control_loop()'s left/right mixing,
        # NOT here too - doing it in both places would cancel out).
        # target_angular sign flip REMOVED (2026-09-17) - it was inverting
        # left/right turns vs. cmd_vel intent (confirmed via teleop testing).
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        self.last_cmd_vel_time = time.monotonic()

    def motion_raw_callback(self, msg):
        self.measured_linear = msg.linear.x * LINEAR_TELEMETRY_CORRECTION
        self.measured_angular = msg.angular.z * ANGULAR_TELEMETRY_CORRECTION
        self.last_motion_time = time.monotonic()

    def odom_raw_callback(self, msg):
        now = time.monotonic()
        left_m, right_m = msg.data[0], msg.data[1]
        if self._last_odom_left_m is not None:
            dt = now - self.last_odom_time
            if dt > 0:
                self.measured_left_speed = (left_m - self._last_odom_left_m) / dt
                self.measured_right_speed = (right_m - self._last_odom_right_m) / dt
        self._last_odom_left_m = left_m
        self._last_odom_right_m = right_m
        self.last_odom_time = now

    # Runs at CONTROL_PERIOD regardless of cmd_vel message rate: reads real
    # measured speed from get_motion_data(), runs the two feedforward+PI
    # loops, mixes them into left/right duty and sends it via set_motor()
    # (bypasses the firmware's own broken speed PID - see constants above).
    def control_loop(self):
        now = time.monotonic()
        dt = now - self._last_control_time
        self._last_control_time = now
        if dt <= 0:
            return

        if now - self.last_cmd_vel_time > CMD_VEL_TIMEOUT:
            self.target_linear = 0.0
            self.target_angular = 0.0

        closed_loop = now - self.last_motion_time <= MOTION_FEEDBACK_TIMEOUT
        if not closed_loop:
            self.get_logger().warn(
                f"No motion_raw for >{MOTION_FEEDBACK_TIMEOUT}s - driving OPEN LOOP "
                "(feedforward only, no speed trim). Run ugv_bringup alongside this "
                "node for closed-loop control: it is the only process allowed to "
                "read the board's serial port, and it republishes the speed "
                "telemetry this node needs.",
                throttle_duration_sec=5.0,
            )

        # Full stop: cut the motors outright instead of letting the PI trim
        # brake against the measured speed.
        if self.target_linear == 0.0 and self.target_angular == 0.0:
            self.linear_ctrl.reset()
            self.angular_ctrl.reset()
            self.car.set_motor(0, 0, 0, 0)
            return

        duty_lin = self.linear_ctrl.update(self.target_linear, self.measured_linear, dt, closed_loop)
        duty_ang = self.angular_ctrl.update(self.target_angular, self.measured_angular, dt, closed_loop)

        # m1=front-left, m2=rear-left, m3=front-right, m4=rear-right (see
        # app_fourwheel.c's Fourwheel_Ctrl). Forward needs NEGATIVE duty on
        # all wheels (confirmed empirically); duty_ang is added/subtracted
        # so that increasing it increases measured_angular, matching a
        # positive target_angular (self-consistent regardless of which
        # physical side duty_ang happens to speed up - only the closed-loop
        # sign matters for stability; flip here if real rotation direction
        # ends up reversed vs. cmd_vel intent, same as the linear note above).
        left = -duty_lin + duty_ang
        right = -duty_lin - duty_ang

        # Reverse-direction breakaway kick - see PER_SIDE_REVERSE_BREAKAWAY_BOOST
        # above. Only trusted while odom_raw is fresh; a side is only boosted
        # while it's commanded BACKWARD (positive duty) and genuinely not
        # moving yet, magnitude-only so the (unknown/irrelevant) sign
        # convention of the raw per-side encoder speed doesn't matter.
        if now - self.last_odom_time <= ODOM_FEEDBACK_TIMEOUT:
            if left > 0 and abs(self.measured_left_speed) < PER_SIDE_BREAKAWAY_SPEED:
                stall = 1.0 - abs(self.measured_left_speed) / PER_SIDE_BREAKAWAY_SPEED
                left += PER_SIDE_REVERSE_BREAKAWAY_BOOST * stall
            if right > 0 and abs(self.measured_right_speed) < PER_SIDE_BREAKAWAY_SPEED:
                stall = 1.0 - abs(self.measured_right_speed) / PER_SIDE_BREAKAWAY_SPEED
                right += PER_SIDE_REVERSE_BREAKAWAY_BOOST * stall

        # Scale both sides down together (preserving the turn ratio) if
        # their sum would exceed the duty limit.
        max_mag = max(abs(left), abs(right), DUTY_LIMIT)
        scale = DUTY_LIMIT / max_mag
        left *= scale
        right *= scale

        self.car.set_motor(int(round(left)), int(round(left)), int(round(right)), int(round(right)))

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
        node.car.set_motor(0, 0, 0, 0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
