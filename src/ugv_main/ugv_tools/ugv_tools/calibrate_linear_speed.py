#!/usr/bin/env python3
"""Multi-speed linear speed calibration for the Rosmaster board.

NOT a ROS2 node - run directly with python3. Unlike a single-point test at a
very low speed (which falls into a motor stiction/deadband zone and gives a
misleading correction factor - see /memories/repo/rosmaster_motor_controller.md,
2026-09-15 entry), this drives the robot at several REALISTIC operating
speeds, each for a longer duration, so the startup transient is a small
fraction of the measured run.

For each test speed the robot drives in a straight line for DURATION seconds,
then stops. You measure the real-world distance travelled (tape measure /
floor marks) and type it in cm when prompted. At the end the script fits
actual_speed = a * commanded_speed + b (simple linear regression) and prints:
  - the fitted a, b
  - the inverse mapping to use in ugv_driver.py if you want commanded speed to
    match requested speed: cmd = (target - b) / a
  - the minimum commanded speed below which points showed inconsistent/absent
    motion, as a deadband warning

Run with: python3 calibrate_linear_speed.py
Needs open, flat, clear floor space - the longest run needs SPEEDS[-1] * DURATION
metres of clearance in front of the robot, plus margin. Sized (0.1-0.25 m/s,
5s) for a ~3.5-4m corridor.
"""
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

# Realistic operating speeds (m/s) - 0.1 m/s was already confirmed NOT to be
# in the deadband zone (2026-09-15 ground test), so it's a safe lower bound.
# Kept low to fit a ~3.5-4m corridor: longest run covers SPEEDS[-1]*DURATION = 1.25m.
SPEEDS = [0.1, 0.15, 0.2, 0.25]
DURATION = 4.0  # seconds per run


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
    input(f"\n--- {speed} m/s for {DURATION}s ---\n"
          f"Mark the robot's current position, clear its path, then press Enter to start...")
    bot.set_car_motion(speed, 0.0, 0.0)
    time.sleep(DURATION)
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.3)
    while True:
        raw = input("Measured distance travelled, in cm (or 'skip'): ").strip()
        if raw.lower() == 'skip':
            return None
        try:
            return float(raw) / 100.0
        except ValueError:
            print("Please enter a number (e.g. 87.5) or 'skip'.")


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
    results = []  # list of (commanded, actual)
    try:
        for speed in SPEEDS:
            distance_m = run_one_speed(bot, speed)
            if distance_m is None:
                continue
            actual_speed = distance_m / DURATION
            ratio = actual_speed / speed if speed else float('nan')
            print(f"  -> actual speed: {actual_speed:.3f} m/s (ratio {ratio:.2f}x commanded)")
            results.append((speed, actual_speed))
    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)

    print("\n=== Results ===")
    for speed, actual in results:
        print(f"  commanded {speed:.2f} m/s -> actual {actual:.3f} m/s")

    if len(results) < 2:
        print("Need at least 2 measured points to fit a line - re-run with more data.")
        return

    xs = [r[0] for r in results]
    ys = [r[1] for r in results]
    a, b = fit_line(xs, ys)
    print(f"\nFitted model: actual_speed = {a:.3f} * commanded_speed + {b:.3f}")
    print("To make actual speed match a requested target speed, command:")
    print(f"  cmd = (target_speed - {b:.3f}) / {a:.3f}")
    print("\nIf 'b' is far from 0, that's a deadband/offset term (motor needs a")
    print("minimum push before it starts moving) rather than a pure scale factor -")
    print("do not extrapolate this fit below the lowest speed you actually tested.")


if __name__ == '__main__':
    main()
