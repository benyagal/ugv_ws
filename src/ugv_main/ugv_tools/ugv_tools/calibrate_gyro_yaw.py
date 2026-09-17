#!/usr/bin/env python3
"""Raw gyro yaw-rate scale calibration for the Rosmaster board's IMU.

NOT a ROS2 node - run directly with python3. Companion to
calibrate_angular_speed.py, but measures the IMU's raw gz reading instead of
the wheel-encoder-based angular speed - these are computed completely
differently in Rosmaster_Lib (physical gyro chip + hardcoded per-chip scale
factor, vs. wheel encoder deltas), so one being calibrated tells you nothing
about the other.

For each run the robot spins in place for DURATION seconds while this script
integrates the raw get_gyroscope_data() gz reading (rad/s) over time to get a
"gyro-measured" total rotation. Before each run, mark the robot's current
heading (e.g. a piece of tape on the floor aligned with the front of the
robot). After it stops, measure the ACTUAL rotation in degrees (protractor,
phone compass app, or estimate against floor marks) and type it in when
prompted - can be more than 360 (e.g. 720 for two full turns). Prefer several
full rotations per run to reduce measurement error as a fraction of the total.

At the end the script prints the average correction factor:
    GYRO_Z_SCALE_CORRECTION = actual_angle / gyro_integrated_angle
to paste into ugv_bringup.py.

Run with: python3 calibrate_gyro_yaw.py
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

SPIN_SPEED = 0.5  # rad/s, commanded via set_car_motion
DURATION = 5.0  # seconds per run
GYRO_POLL_PERIOD = 0.02  # seconds between gz samples while integrating

# Same rationale as ugv_bringup.py's own calibration - average out gz noise
# while stationary before spinning, so that noise doesn't bias the integral.
BIAS_SAMPLES = 200


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


def measure_gyro_bias(bot):
    print(f"Measuring gz bias from {BIAS_SAMPLES} stationary samples - keep the robot still...")
    samples = []
    for _ in range(BIAS_SAMPLES):
        _, _, gz = bot.get_gyroscope_data()
        samples.append(gz)
        time.sleep(GYRO_POLL_PERIOD)
    bias = sum(samples) / len(samples)
    print(f"  gz bias: {bias:.5f} rad/s")
    return bias


def run_one_spin(bot, gz_bias):
    input(f"\n--- spin {SPIN_SPEED} rad/s for {DURATION}s ---\n"
          f"Mark the robot's current heading, clear space around it, then press Enter to start...")

    integrated_angle = 0.0
    bot.set_car_motion(0.0, 0.0, SPIN_SPEED)
    start = time.monotonic()
    last = start
    while time.monotonic() - start < DURATION:
        now = time.monotonic()
        dt = now - last
        last = now
        _, _, gz = bot.get_gyroscope_data()
        integrated_angle += (gz - gz_bias) * dt
        time.sleep(GYRO_POLL_PERIOD)
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.3)

    print(f"  gyro-integrated rotation: {math.degrees(integrated_angle):.1f} deg")
    while True:
        raw = input("Measured ACTUAL rotation, in degrees (can be >360, or 'skip'): ").strip()
        if raw.lower() == 'skip':
            return None
        try:
            actual_angle = math.radians(float(raw))
            # Preserve the actual measured direction/sign vs. the integrated
            # value's sign, so the ratio comes out positive for a consistent
            # (even if mislabeled-sign) scale error.
            if integrated_angle < 0:
                actual_angle = -abs(actual_angle)
            return actual_angle, integrated_angle
        except ValueError:
            print("Please enter a number (e.g. 720) or 'skip'.")


def main():
    bot = connect()
    ratios = []
    try:
        gz_bias = measure_gyro_bias(bot)
        for _ in range(3):
            result = run_one_spin(bot, gz_bias)
            if result is None:
                continue
            actual_angle, integrated_angle = result
            if integrated_angle == 0:
                print("  integrated angle was 0 - skipping this run.")
                continue
            ratio = actual_angle / integrated_angle
            print(f"  -> ratio (actual/gyro): {ratio:.3f}")
            ratios.append(ratio)
    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)

    if not ratios:
        print("No usable runs - re-run and provide at least one measured angle.")
        return

    avg_ratio = sum(ratios) / len(ratios)
    print("\n=== Result ===")
    for r in ratios:
        print(f"  ratio: {r:.3f}")
    print(f"\nGYRO_Z_SCALE_CORRECTION = {avg_ratio:.3f}")
    print("Paste this value into ugv_bringup.py's GYRO_Z_SCALE_CORRECTION constant.")


if __name__ == '__main__':
    main()
