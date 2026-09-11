#!/usr/bin/env python3
"""Standalone diagnostic tool for the new Yahboom Rosmaster (STM32F103RCT6) board.

NOT a ROS2 node - run this directly with plain python3 once Rosmaster_Lib is
installed on the Jetson, before touching ugv_bringup.py/ugv_driver.py. The goal
is to empirically determine facts we don't yet know from the docs:

  1. What serial port the board actually enumerates as (default '/dev/myserial'
     may not exist - try common candidates or ask the user to run
     `ls -l /dev/ttyUSB* /dev/ttyACM* /dev/myserial 2>/dev/null`).
  2. What physical units get_accelerometer_data()/get_gyroscope_data() return
     (raw register counts? g's? deg/s?) - compare against gravity (~9.8 m/s^2
     total magnitude while stationary) and known rotations.
  3. What set_car_motion(v_x, v_y, v_z) actually does at small values - is
     v_x a fraction of max speed, or literal m/s? Compare commanded value
     against get_motion_data() feedback and/or measured real-world distance.
  4. Whether get_battery_voltage() reports plausible pack voltage directly.

Run with: python3 test_rosmaster_board.py
Stop with Ctrl+C. Keep the robot's wheels OFF THE GROUND (propped up) the
first time you run the motion test, in case the sign/scale is unexpected.
"""
import sys
import time

try:
    from Rosmaster_Lib import Rosmaster
except ImportError:
    print("Rosmaster_Lib not importable - install it first (see Rosmaster_install.txt)")
    sys.exit(1)

# Candidate ports to try, in order, since the actual enumeration on this
# board/Jetson combo is not yet confirmed.
CANDIDATE_PORTS = ['/dev/myserial', '/dev/ttyUSB0', '/dev/ttyACM0', '/dev/ttyTHS1']

# Must match ugv_bringup.py/ugv_driver.py's CAR_TYPE - the firmware selects
# its wheel-mixing algorithm from this value on every set_car_motion() call,
# so testing with the wrong car_type would test the wrong kinematics.
CAR_TYPE = 4


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


def print_sensor_snapshot(bot):
    ax, ay, az = bot.get_accelerometer_data()
    gx, gy, gz = bot.get_gyroscope_data()
    mx, my, mz = bot.get_magnetometer_data()
    vx, vy, vz = bot.get_motion_data()
    voltage = bot.get_battery_voltage()
    print(
        f"accel=({ax:.3f},{ay:.3f},{az:.3f}) "
        f"gyro=({gx:.3f},{gy:.3f},{gz:.3f}) "
        f"mag=({mx:.3f},{my:.3f},{mz:.3f}) "
        f"motion=({vx:.3f},{vy:.3f},{vz:.3f}) "
        f"voltage={voltage:.2f}"
    )


def main():
    bot = connect()

    print("\n--- Stationary sensor snapshot (robot must not be moving) ---")
    for _ in range(5):
        print_sensor_snapshot(bot)
        time.sleep(0.5)

    input(
        "\nProp the wheels up off the ground, then press Enter to run a "
        "small forward motion test (Ctrl+C to skip)..."
    )
    try:
        print("Commanding set_car_motion(0.2, 0, 0) for 3 seconds...")
        bot.set_car_motion(0.2, 0, 0)
        for _ in range(6):
            print_sensor_snapshot(bot)
            time.sleep(0.5)
    finally:
        bot.set_car_motion(0, 0, 0)
        print("Stopped.")


if __name__ == '__main__':
    main()
