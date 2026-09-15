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
# Raised from the raw calibration fit (29.9): real testing (2026-09-15) found
# one wheel (front-right) needs more duty than the others to break static
# friction - at 0.3 rad/s target the old offset gave ~37.6 duty, too low for
# that wheel to start immediately, causing a multi-second ramp-up and
# undershoot; at 0.6 rad/s (~45 duty) all wheels started at once and tracked
# ~90% of target. This offset ensures even low targets clear that threshold.
ANGULAR_FF_OFFSET = 38.0  # duty needed to overcome stiction/deadband

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

    def __init__(self, ff_slope, ff_offset, kp, ki):
        self.ff_slope = ff_slope
        self.ff_offset = ff_offset
        self.kp = kp
        self.ki = ki
        self.integral = 0.0

    def reset(self):
        self.integral = 0.0

    def update(self, target, measured, dt):
        if target == 0.0:
            # Don't hold an integral term while stopped - avoids any lurch
            # the next time a nonzero target is commanded.
            self.reset()
            return 0.0

        sign = 1.0 if target > 0 else -1.0
        feedforward = sign * (self.ff_slope * abs(target) + self.ff_offset)

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
        self.car.create_receive_threading()

        # Only log the "not implemented" servo/LED warnings once each,
        # instead of spamming on every message.
        self._warned_joint_states = False
        self._warned_led_ctrl = False

        # Custom closed-loop speed control state (see constants above).
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_vel_time = time.monotonic()
        self._last_control_time = time.monotonic()
        self.linear_ctrl = SpeedPIController(LINEAR_FF_SLOPE, LINEAR_FF_OFFSET, LINEAR_KP, LINEAR_KI)
        self.angular_ctrl = SpeedPIController(ANGULAR_FF_SLOPE, ANGULAR_FF_OFFSET, ANGULAR_KP, ANGULAR_KI)
        self.control_timer = self.create_timer(CONTROL_PERIOD, self.control_loop)

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

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
        self.target_linear = msg.linear.x
        self.target_angular = -msg.angular.z
        self.last_cmd_vel_time = time.monotonic()

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

        vx, _vy, vz = self.car.get_motion_data()
        measured_linear = vx * LINEAR_TELEMETRY_CORRECTION
        measured_angular = vz * ANGULAR_TELEMETRY_CORRECTION

        duty_lin = self.linear_ctrl.update(self.target_linear, measured_linear, dt)
        duty_ang = self.angular_ctrl.update(self.target_angular, measured_angular, dt)

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
