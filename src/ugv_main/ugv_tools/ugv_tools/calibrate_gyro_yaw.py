#!/usr/bin/env python3
"""Raw gyro yaw-rate scale calibration for the Rosmaster board's IMU.

NOT a ROS2 node - run directly with python3. Companion to
calibrate_angular_speed.py, but measures the IMU's raw gz reading instead of
the wheel-encoder-based angular speed - these are computed completely
differently in Rosmaster_Lib (physical gyro chip + hardcoded per-chip scale
factor, vs. wheel encoder deltas), so one being calibrated tells you nothing
about the other.

For each run (see RUNS below - deliberately varying speed/duration, so the
tested rotation amount differs run to run) the robot spins in place while
this script integrates the raw get_gyroscope_data() gz reading (rad/s) over
time to get a "gyro-measured" total rotation. Before each run, mark the
robot's current heading (e.g. a piece of tape on the floor aligned with the
front of the robot). After it stops, measure the ACTUAL rotation in degrees
(protractor, phone compass app, or estimate against floor marks) and type it
in when prompted - can be more than 360 (e.g. 720 for two full turns).

At the end the script fits actual_angle = scale * gyro_integrated_angle
(least squares through the origin, so larger-angle runs are naturally
weighted more heavily) and prints:
    GYRO_Z_SCALE_CORRECTION = scale
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

# (speed_rad_s, duration_s) per run - deliberately varied nominal rotation
# amounts (not just repeating the same one 3x), so we can check the
# actual/gyro ratio stays constant across different angles (confirming a
# clean scale error) rather than masking a non-proportional error, and so
# the larger-angle runs (where your manual measurement error matters less)
# can be weighted more heavily in the final fit.
RUNS = [
    (0.4, 5.0),   # ~2.0 rad (~115 deg)
    (0.4, 10.0),  # ~4.0 rad (~229 deg)
    (0.6, 10.0),  # ~6.0 rad (~344 deg)
    (0.6, 15.0),  # ~9.0 rad (~516 deg, more than a full turn)
]
GYRO_POLL_PERIOD = 0.02  # seconds between gz samples while integrating

# Same wheel-encoder telemetry correction already validated in ugv_driver.py
# (see /memories/repo/rosmaster_motor_controller.md) - printed as an extra,
# independent cross-check alongside the gyro-integrated angle per run, since
# it's already known to be fairly accurate.
ANGULAR_TELEMETRY_CORRECTION = 0.65

# Same rationale as ugv_bringup.py's own calibration - average out gz noise
# while stationary before spinning, so that noise doesn't bias the integral.
BIAS_SAMPLES = 2000


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


def run_one_spin(bot, gz_bias, speed, duration):
    input(f"\n--- spin {speed} rad/s for {duration}s ---\n"
          f"Mark the robot's current heading, clear space around it, then press Enter to start...")

    integrated_angle = 0.0
    wheel_integrated_angle = 0.0
    bot.set_car_motion(0.0, 0.0, speed)
    start = time.monotonic()
    last = start
    while time.monotonic() - start < duration:
        now = time.monotonic()
        dt = now - last
        last = now
        _, _, gz = bot.get_gyroscope_data()
        integrated_angle += (gz - gz_bias) * dt
        _, _, vz = bot.get_motion_data()
        wheel_integrated_angle += vz * ANGULAR_TELEMETRY_CORRECTION * dt
        time.sleep(GYRO_POLL_PERIOD)
    bot.set_car_motion(0.0, 0.0, 0.0)
    time.sleep(0.3)

    print(f"  gyro-integrated rotation:  {math.degrees(integrated_angle):.1f} deg")
    print(f"  wheel-integrated rotation: {math.degrees(wheel_integrated_angle):.1f} deg")
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
    points = []  # (actual_angle, integrated_angle) pairs
    try:
        gz_bias = measure_gyro_bias(bot)
        for speed, duration in RUNS:
            result = run_one_spin(bot, gz_bias, speed, duration)
            if result is None:
                continue
            actual_angle, integrated_angle = result
            if integrated_angle == 0:
                print("  integrated angle was 0 - skipping this run.")
                continue
            ratio = actual_angle / integrated_angle
            print(f"  -> ratio (actual/gyro): {ratio:.3f}")
            points.append((actual_angle, integrated_angle))
    except KeyboardInterrupt:
        pass
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)

    if not points:
        print("No usable runs - re-run and provide at least one measured angle.")
        return

    print("\n=== Result ===")
    for actual_angle, integrated_angle in points:
        print(f"  actual={math.degrees(actual_angle):.1f}deg  gyro={math.degrees(integrated_angle):.1f}deg  "
              f"ratio={actual_angle / integrated_angle:.3f}")

    # Least-squares fit of actual = scale * integrated, forced through the
    # origin (0 rotation should always measure as 0) - this naturally
    # weights larger-angle runs more heavily than a plain average of
    # per-run ratios would, since your measurement error matters less
    # relative to a bigger angle.
    numerator = sum(a * g for a, g in points)
    denominator = sum(g * g for _, g in points)
    scale = numerator / denominator
    print(f"\nGYRO_Z_SCALE_CORRECTION = {scale:.3f}")
    print("Paste this value into ugv_bringup.py's GYRO_Z_SCALE_CORRECTION constant.")


if __name__ == '__main__':
    main()
