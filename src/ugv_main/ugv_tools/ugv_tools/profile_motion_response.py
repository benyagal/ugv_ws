#!/usr/bin/env python3
"""Time-series profiling of the robot's real speed response to one command.

NOT a ROS2 node - run directly with python3. calibrate_linear_speed.py only
measures a single total-distance-over-fixed-time average, which conflates the
acceleration ramp-up with the steady-state cruise speed - this is why 2s and
4s tests at the SAME commanded speed gave very different averages (2026-09-15,
see /memories/repo/rosmaster_motor_controller.md). This script instead polls
get_motion_data() at high rate during a single longer run, so you can see the
actual shape of the response: how long the ramp-up takes, and what speed it
settles to (or whether it saturates near a hardware max).

At the end it also asks for the real-world measured distance, and compares it
to the distance obtained by integrating get_motion_data()'s reported vx over
time - if those roughly agree, get_motion_data() itself can be trusted as a
live speed sensor (useful for future closed-loop control), just possibly with
a scale correction.

Run with: python3 profile_motion_response.py [speed_m_s] [duration_s]
Defaults to 0.15 m/s for 6s if not given. Needs ~ speed*duration metres of
clear floor space ahead of the robot, plus margin.
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


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

    bot = connect()
    input(f"\nWill drive at {speed} m/s (commanded) for {duration}s, polling every "
          f"{POLL_PERIOD}s.\nMark the robot's current position, clear its path, "
          f"then press Enter to start...")

    samples = []  # (t, vx)
    start = time.monotonic()
    # set_car_motion's raw sign drives backward on our board - negate so
    # 'speed' means forward, matching ugv_driver.py's convention.
    bot.set_car_motion(-speed, 0.0, 0.0)
    try:
        while True:
            t = time.monotonic() - start
            if t >= duration:
                break
            vx, _vy, _vz = bot.get_motion_data()
            samples.append((t, vx))
            print(f"  t={t:5.2f}s  vx={vx:+.3f} m/s")
            time.sleep(POLL_PERIOD)
    finally:
        bot.set_car_motion(0.0, 0.0, 0.0)

    if len(samples) < 2:
        print("Not enough samples collected - try a longer duration.")
        return

    # Trapezoidal integration of |vx| over time -> distance implied by telemetry.
    integrated_distance = 0.0
    for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
        integrated_distance += 0.5 * (abs(v0) + abs(v1)) * (t1 - t0)

    # Steady-state estimate: average of the last third of samples.
    tail = samples[len(samples) * 2 // 3:]
    steady_state = sum(abs(v) for _t, v in tail) / len(tail)

    print(f"\nIntegrated distance from get_motion_data(): {integrated_distance * 100:.1f} cm")
    print(f"Estimated steady-state speed (last third of run): {steady_state:.3f} m/s")

    while True:
        raw = input("\nMeasured REAL distance travelled, in cm (or 'skip'): ").strip()
        if raw.lower() == 'skip':
            return
        try:
            real_distance_m = float(raw) / 100.0
            break
        except ValueError:
            print("Please enter a number (e.g. 87.5) or 'skip'.")

    if integrated_distance > 0:
        ratio = real_distance_m / integrated_distance
        print(f"Real distance / telemetry-integrated distance = {ratio:.2f}x")
        print("(close to 1.0 means get_motion_data() is a trustworthy live speed sensor)")


if __name__ == '__main__':
    main()
