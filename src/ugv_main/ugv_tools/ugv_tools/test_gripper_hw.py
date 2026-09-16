#!/usr/bin/env python3
"""Interactive hardware test tool for the NEW gripper hardware layout
(2026-09-15) - supersedes both ugv_gripper/gripper_node.py (PCA9685 servos +
Jetson GPIO relay/switch) and gripper_arduino_node.py (external Arduino
running Jetson_to_Arduino2.ino): there is no Arduino and no PCA9685 anymore.

Current hardware (per user, all polarity/behavior UNVERIFIED - that's what
this tool is for):
  - S1: 360 degree (continuous rotation) servo, on the Yahboom motor
    controller board (v3) - driven via Rosmaster_Lib's set_pwm_servo(1, ...)
  - S2, S3: 180 degree servos, same motor controller board, channels 2/3
  - Linear DC motor (relay-driven): wired directly to the Jetson's own GPIO
    header (no Arduino/PCA9685 in between) - physical pin 29 (GPIO01)
    energizes the relay that drives it OUT, physical pin 31 (GPIO11)
    energizes the relay that drives it IN
  - Homing microswitch: physical pin 7 (GPIO09), should stop S1's outward
    motion when triggered

Usage:
  python3 test_gripper_hw.py s1 <angle 0-180> [duration_s]
  python3 test_gripper_hw.py s2 <angle 0-180> [duration_s]
  python3 test_gripper_hw.py s3 <angle 0-180> [duration_s]
  python3 test_gripper_hw.py relay <out|in|stop> [duration_s]
  python3 test_gripper_hw.py switch [duration_s]

'switch' just polls and prints the raw pin level/edges, to find its resting
state and confirm wiring before trusting it as a safety cutoff. 's1' polls
the same switch in the background and forces the servo back to
S1_STOP_ANGLE the moment it reads as pressed - use --active-high if
'switch' shows it resting HIGH and going LOW when pressed is wrong (default
assumes a pulled-up switch that reads LOW when pressed).

KNOWN ISSUE (found 2026-09-15): Jetson.GPIO ignores setup()'s
pull_up_down parameter on this carrier board (it prints a UserWarning
saying so), so SWITCH_PIN has NO defined idle level without an external
pull resistor wired to it - 'switch' read PRESSED constantly, at rest,
without the button touched. Until an external pull-up/down resistor is
added to the wiring, pass --ignore-switch to 's1' to test the servo
without the (currently unreliable) safety cutoff.
"""
import argparse
import subprocess
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("Rosmaster_Lib not importable - install it first (see Rosmaster_install.txt)")
    sys.exit(1)

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("Jetson.GPIO not importable - install it first (pip install Jetson.GPIO)")
    sys.exit(1)

CANDIDATE_PORTS = ['/dev/ttyUSB0', '/dev/myserial', '/dev/ttyACM0', '/dev/ttyTHS1']
CAR_TYPE = 4

RELAY_OUT_PIN = 29  # GPIO01 - energizes the relay driving the linear motor OUT
RELAY_IN_PIN = 31   # GPIO11 - energizes the relay driving the linear motor IN
SWITCH_PIN = 33      # GPIO09 - homing microswitch, should stop S1's OUT motion

S1_STOP_ANGLE = 90  # presumed neutral/stop point for the continuous servo - verify with the 's1' command

# S2 is mounted mirrored relative to S3 (confirmed empirically 2026-09-15:
# S3's raw angle was correct, S2's was inverted) - flip it here so callers
# always pass the logical/intended angle for both.
MIRRORED_SERVO_IDS = (2,)

# On this carrier board, physical pins 29/31 (BOARD numbering) boot up with
# their pinmux set to input, so GPIO.output() silently has no effect on the
# actual pin voltage even though it reports success (same issue already
# found/fixed for ugv_gripper's relay pins - see bringup_gripper.launch.py).
# Re-applied here since this standalone tool doesn't go through that launch
# file. Only lasts until next reboot; re-derive via the "sudo busybox
# devmem ..." command Jetson.GPIO's own warning prints if pins ever change.
PINMUX_FIXUPS = {
    29: ['busybox', 'devmem', '0x2430068', 'w', '0x8'],
    31: ['busybox', 'devmem', '0x2430070', 'w', '0x8'],
}


def connect_rosmaster():
    for port in CANDIDATE_PORTS:
        try:
            bot = Rosmaster(car_type=CAR_TYPE, com=port, debug=False)
            bot.create_receive_threading()
            time.sleep(0.5)
            version = bot.get_version()
            print(f"Connected on {port}, MCU firmware version: {version}")
            return bot
        except Exception as e:
            print(f"  {port}: failed ({e})")
    print("Could not connect on any candidate port - check wiring/port name.")
    sys.exit(1)


def switch_pressed(active_low):
    level = GPIO.input(SWITCH_PIN)
    return (level == GPIO.LOW) if active_low else (level == GPIO.HIGH)


def cmd_s1(angle, duration, active_low, ignore_switch):
    bot = connect_rosmaster()
    print(f"Driving S1 to angle={angle} for up to {duration}s (Ctrl+C to stop early)...")
    if ignore_switch:
        print("--ignore-switch given - NOT watching the microswitch this run.")
    else:
        GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP if active_low else GPIO.PUD_DOWN)
        print("Watching the microswitch in the background - S1 will be forced to stop if it's pressed.")
    bot.set_pwm_servo(1, angle)
    start = time.monotonic()
    stopped_by_switch = False
    try:
        while time.monotonic() - start < duration:
            if not ignore_switch and switch_pressed(active_low):
                print("Microswitch pressed - stopping S1.")
                stopped_by_switch = True
                break
            time.sleep(0.02)
    finally:
        bot.set_pwm_servo(1, S1_STOP_ANGLE)
    print(f"S1 stopped{' (by microswitch)' if stopped_by_switch else ' (duration elapsed)'}.")


def cmd_s180(servo_id, angle, duration):
    bot = connect_rosmaster()
    physical_angle = 180 - angle if servo_id in MIRRORED_SERVO_IDS else angle
    print(f"Driving S{servo_id} to logical angle={angle} (physical={physical_angle}), holding for {duration}s...")
    bot.set_pwm_servo(servo_id, physical_angle)
    time.sleep(duration)
    print("Done (servo holds its last commanded position - send another angle, e.g. 90, to re-center).")


def cmd_relay(direction, duration):
    for fixup_cmd in PINMUX_FIXUPS.values():
        subprocess.run(fixup_cmd, check=False)
    GPIO.setup(RELAY_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RELAY_IN_PIN, GPIO.OUT, initial=GPIO.LOW)
    if direction == 'stop':
        print("Stopping (de-energizing both relay pins).")
        GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
        GPIO.output(RELAY_IN_PIN, GPIO.LOW)
        return
    if direction == 'out':
        print(f"Energizing OUT relay (pin {RELAY_OUT_PIN}) for {duration}s...")
        GPIO.output(RELAY_IN_PIN, GPIO.LOW)
        GPIO.output(RELAY_OUT_PIN, GPIO.HIGH)
    else:
        print(f"Energizing IN relay (pin {RELAY_IN_PIN}) for {duration}s...")
        GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
        GPIO.output(RELAY_IN_PIN, GPIO.HIGH)
    try:
        time.sleep(duration)
    finally:
        GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
        GPIO.output(RELAY_IN_PIN, GPIO.LOW)
        print("Relays de-energized.")


def cmd_switch(duration, active_low):
    GPIO.setup(SWITCH_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP if active_low else GPIO.PUD_DOWN)
    print(f"Polling microswitch on pin {SWITCH_PIN} for {duration}s (press it to test)...")
    start = time.monotonic()
    last = None
    while time.monotonic() - start < duration:
        pressed = switch_pressed(active_low)
        if pressed != last:
            print(f"  t={time.monotonic() - start:5.2f}s  switch {'PRESSED' if pressed else 'released'}")
            last = pressed
        time.sleep(0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='mode', required=True)

    p_s1 = sub.add_parser('s1', help='drive the 360 degree servo (motor controller channel 1)')
    p_s1.add_argument('angle', type=int, help='0-180 (90 is presumed stop - verify!)')
    p_s1.add_argument('duration', type=float, nargs='?', default=3.0)
    p_s1.add_argument('--active-high', dest='active_low', action='store_false', default=True,
                       help="microswitch reads HIGH when pressed (default assumes LOW-when-pressed, pulled up)")
    p_s1.add_argument('--ignore-switch', action='store_true',
                       help="don't watch the microswitch at all (use while its wiring/pull resistor is unconfirmed)")

    for name in ('s2', 's3'):
        p = sub.add_parser(name, help=f'drive the 180 degree servo (motor controller channel {name[1]})')
        p.add_argument('angle', type=int, help='0-180')
        p.add_argument('duration', type=float, nargs='?', default=1.0)

    p_relay = sub.add_parser('relay', help='drive the linear actuator relay via Jetson GPIO')
    p_relay.add_argument('direction', choices=['out', 'in', 'stop'])
    p_relay.add_argument('duration', type=float, nargs='?', default=2.0)

    p_switch = sub.add_parser('switch', help='poll the homing microswitch and print its state on every edge')
    p_switch.add_argument('duration', type=float, nargs='?', default=10.0)
    p_switch.add_argument('--active-high', dest='active_low', action='store_false', default=True,
                           help="assume/report HIGH as pressed instead of the default LOW-when-pressed")

    args = parser.parse_args()

    GPIO.setmode(GPIO.BOARD)
    try:
        if args.mode == 's1':
            cmd_s1(args.angle, args.duration, args.active_low, args.ignore_switch)
        elif args.mode in ('s2', 's3'):
            cmd_s180(int(args.mode[1]), args.angle, args.duration)
        elif args.mode == 'relay':
            cmd_relay(args.direction, args.duration)
        elif args.mode == 'switch':
            cmd_switch(args.duration, args.active_low)
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
