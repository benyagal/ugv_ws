#!/usr/bin/env python3
"""Test whether retuning the firmware's motor PID, or bypassing it with raw
PWM, gives a proportional/controllable real speed response.

Context (see /memories/repo/rosmaster_motor_controller.md): profile_motion_
response.py showed that set_car_motion() saturates to the SAME real-world
speed (~0.87-0.89 m/s telemetry / ~0.72 m/s real) for ANY commanded value from
~0.06 up to 2.0 m/s - there is no usable proportional range. The firmware
source (app_motion.c/app_fourwheel.c) sets the low-level motor PID's target
directly from the raw commanded mm/s, but computes its feedback from encoder
pulses via hard-coded FOURWHEEL_CIRCLE_MM=215.2 / ENCODER_CIRCLE_330=1320
constants tuned for Yahboom's own motor - if our actual motor/wheel differs
(as it does), this mismatches target vs. feedback and/or makes the PID
saturate almost immediately. This script tests the two fixes exposed by
Rosmaster_Lib without needing a firmware reflash:
  1. set_pid_param(kp, ki, kd) - retune the closed-loop gains (--kp/--ki/--kd)
  2. set_motor(...) - raw PWM duty [-100,100], bypasses the encoder PID
     entirely (--raw)

Goal is narrowly: find a control path where the REAL speed varies smoothly
and predictably with the commanded value, so TEB/Nav2 velocity commands
actually get followed - not a full PID retune exercise.

Run with: python3 tune_motor_pid.py <value> [duration_s] [--kp K --ki K --kd K] [--raw] [--spin]
  <value> is m/s for normal mode, or PWM duty [-100,100] for --raw mode.
  --spin (only with --raw) drives left/right sides in opposite directions to
  test pure rotation, since the firmware's set_car_motion() angular control
  is separately known to be ~5x too fast/uncorrected - need to know if raw
  PWM rotation is smooth/proportional before designing a custom controller.
  Motor channel order (from app_fourwheel.c's Fourwheel_Ctrl): m1=front-left,
  m2=rear-left, m3=front-right, m4=rear-right.
"""
import argparse
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("Rosmaster_Lib not importable - install it first (see Rosmaster_install.txt)")
    sys.exit(1)

CANDIDATE_PORTS = ['/dev/ttyUSB0', '/dev/myserial', '/dev/ttyACM0', '/dev/ttyTHS1']
CAR_TYPE = 4
POLL_PERIOD = 0.1  # seconds between get_motion_data() reads


def connect():
    for port in CANDIDATE_PORTS:
        try:
            bot = Rosmaster(car_type=CAR_TYPE, com=port, debug=False)
            bot.create_receive_threading()
            bot.set_auto_report_state(True, forever=False)
            time.sleep(0.5)
            version = bot.get_version()
            print(f"Connected on {port}, MCU firmware version: {version}")
            return bot
        except Exception as e:
            print(f"  {port}: failed ({e})")
    print("Could not connect on any candidate port - check wiring/port name.")
    sys.exit(1)


def profile(bot, start_cmd, stop_cmd, duration, field='vx'):
    """Run start_cmd(), poll get_motion_data() for duration seconds, then stop_cmd().

    field selects which of get_motion_data()'s (vx, vy, vz) to track/print.
    """
    field_index = {'vx': 0, 'vy': 1, 'vz': 2}[field]
    unit = 'm/s' if field != 'vz' else 'rad/s'
    samples = []  # (t, value)
    start = time.monotonic()
    start_cmd()
    try:
        while True:
            t = time.monotonic() - start
            if t >= duration:
                break
            value = bot.get_motion_data()[field_index]
            samples.append((t, value))
            print(f"  t={t:5.2f}s  {field}={value:+.3f} {unit}")
            time.sleep(POLL_PERIOD)
    finally:
        stop_cmd()

    if len(samples) < 2:
        print("Not enough samples collected - try a longer duration.")
        return

    tail = samples[len(samples) * 2 // 3:]
    steady_state = sum(abs(v) for _t, v in tail) / len(tail)
    print(f"Estimated steady-state {field} (last third of run): {steady_state:.3f} {unit}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('value', type=float, help='m/s (normal) or PWM duty [-100,100] (--raw)')
    parser.add_argument('duration', type=float, nargs='?', default=6.0)
    parser.add_argument('--kp', type=float, default=None)
    parser.add_argument('--ki', type=float, default=None)
    parser.add_argument('--kd', type=float, default=None)
    parser.add_argument('--raw', action='store_true', help='use set_motor() raw PWM instead of set_car_motion()')
    parser.add_argument('--spin', action='store_true', help='pure rotation test (left/right differential), requires --raw')
    args = parser.parse_args()

    if args.spin and not args.raw:
        print("--spin is only implemented for --raw mode.")
        sys.exit(1)

    bot = connect()

    if args.kp is not None or args.ki is not None or args.kd is not None:
        if None in (args.kp, args.ki, args.kd):
            print("Must give --kp, --ki and --kd together.")
            sys.exit(1)
        print(f"Setting PID gains: kp={args.kp} ki={args.ki} kd={args.kd} (temporary, forever=False)")
        bot.set_pid_param(args.kp, args.ki, args.kd, forever=False)
        time.sleep(0.2)

    if args.raw and args.spin:
        duty = args.value
        input(f"\nWill spin in place at RAW PWM duty {duty} per side (of [-100,100]) for "
              f"{args.duration}s.\nMark the robot's current heading, clear space around it, "
              f"then press Enter to start...")
        # m1/m2=left, m3/m4=right (see Fourwheel_Ctrl) - opposite signs to spin in place.
        profile(bot,
                 start_cmd=lambda: bot.set_motor(-duty, -duty, duty, duty),
                 stop_cmd=lambda: bot.set_motor(0, 0, 0, 0),
                 duration=args.duration,
                 field='vz')
    elif args.raw:
        duty = args.value
        input(f"\nWill drive at RAW PWM duty {duty} (of [-100,100]) for {args.duration}s.\n"
              f"Mark the robot's current position, clear its path, then press Enter to start...")
        # Same sign convention as set_car_motion() usage elsewhere: negate so
        # positive 'value' means forward.
        profile(bot,
                 start_cmd=lambda: bot.set_motor(-duty, -duty, -duty, -duty),
                 stop_cmd=lambda: bot.set_motor(0, 0, 0, 0),
                 duration=args.duration)
    else:
        speed = args.value
        input(f"\nWill drive at {speed} m/s (commanded) for {args.duration}s.\n"
              f"Mark the robot's current position, clear its path, then press Enter to start...")
        profile(bot,
                 start_cmd=lambda: bot.set_car_motion(-speed, 0.0, 0.0),
                 stop_cmd=lambda: bot.set_car_motion(0.0, 0.0, 0.0),
                 duration=args.duration)

    prompt = ("\nMeasured REAL rotation, in degrees (or 'skip'): " if (args.raw and args.spin)
              else "\nMeasured REAL distance travelled, in cm (or 'skip'): ")
    while True:
        raw = input(prompt).strip()
        if raw.lower() == 'skip':
            return
        try:
            float(raw)
            return
        except ValueError:
            print("Please enter a number (or 'skip').")


if __name__ == '__main__':
    main()
