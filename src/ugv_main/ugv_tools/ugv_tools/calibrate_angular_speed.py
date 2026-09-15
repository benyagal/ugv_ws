#!/usr/bin/env python3
"""Multi-speed angular speed calibration for the Rosmaster board.

NOT a ROS2 node - run directly with python3. Companion to
calibrate_linear_speed.py - same rationale applies (a single data point at one
speed can be misleading due to motor deadband/stiction, or simply because the
mismatch factor is not constant across speeds). See
/memories/repo/rosmaster_motor_controller.md, 2026-09-15 entries.

For each test speed the robot rotates in place for DURATION seconds, then
stops. Before each run, mark the robot's current heading (e.g. a piece of tape
on the floor aligned with the front of the robot). After it stops, measure the
actual rotation in degrees (protractor, phone compass app, or estimate against
floor marks) and type it in when prompted - can be more than 360 (e.g. 720 for
two full turns). At the end the script fits
actual_rad_s = a * commanded_rad_s + b (simple linear regression) and prints
the inverse mapping to use in ugv_driver.py.

Run with: python3 calibrate_angular_speed.py
Needs open floor space with room for the robot to spin freely in place.
"""
import math
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("Rosmaster_Lib not importable - install it first (see Rosmaster_install.txt)")
    sys.exit(1)

# Same candidates/car_type as test_rosmaster_board.py / ugv_driver.py.
CANDIDATE_PORTS = ['/dev/ttyUSB0', '/dev/myserial', '/dev/ttyACM0', '/dev/ttyTHS1']
CAR_TYPE = 4

# Realistic operating angular speeds (rad/s).
SPEEDS = [0.3, 0.5, 0.8, 1.0]
DURATION = 5.0  # seconds per run


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


def run_one_speed(bot, speed):
    input(f"\n--- {speed} rad/s for {DURATION}s ---\n"
          f"Mark the robot's current heading, clear space around it, then press Enter to start...")
    bot.set_car_motion(0.0, 0.0, speed)
    time.sleep(DURATION)
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.3)
    while True:
        raw = input("Measured rotation, in degrees (can be >360, or 'skip'): ").strip()
        if raw.lower() == 'skip':
            return None
        try:
            return math.radians(float(raw))
        except ValueError:
            print("Please enter a number (e.g. 720) or 'skip'.")


def fit_line(xs, ys):
    """Least-squares fit of ys = a*xs + b, no numpy dependency."""
    n = len(xs)
    mean_x = sum(xs) / n
    mean_y = sum(ys) / n
    num = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, ys))
    den = sum((x - mean_x) ** 2 for x in xs)
    a = num / den
    b = mean_y - a * mean_x
    return a, b


def main():
    bot = connect()
    results = []  # list of (commanded_rad_s, actual_rad_s)
    try:
        for speed in SPEEDS:
            angle_rad = run_one_speed(bot, speed)
            if angle_rad is None:
                continue
            actual_speed = angle_rad / DURATION
            ratio = actual_speed / speed if speed else float('nan')
            print(f"  -> actual speed: {actual_speed:.3f} rad/s (ratio {ratio:.2f}x commanded)")
            results.append((speed, actual_speed))
    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)

    print("\n=== Results ===")
    for speed, actual in results:
        print(f"  commanded {speed:.2f} rad/s -> actual {actual:.3f} rad/s")

    if len(results) < 2:
        print("Need at least 2 measured points to fit a line - re-run with more data.")
        return

    xs = [r[0] for r in results]
    ys = [r[1] for r in results]
    a, b = fit_line(xs, ys)
    print(f"\nFitted model: actual_rad_s = {a:.3f} * commanded_rad_s + {b:.3f}")
    print("To make actual rotation rate match a requested target, command:")
    print(f"  cmd = (target_rad_s - {b:.3f}) / {a:.3f}")
    print("\nDo not extrapolate this fit below the lowest speed you actually tested.")


if __name__ == '__main__':
    main()
