#!/usr/bin/env python3
"""Interactive REPL for exercising the gripper hardware (S1/S2/S3 servos,
linear actuator relay, homing microswitch) without reconnecting to the
Rosmaster board for every single command.

KNOWN ISSUE (see test_gripper_hw.py / test_switch_pin.py): SWITCH_PIN has
no working pull resistor on this carrier board, so it can read PRESSED at
rest with no button touched. Until that wiring is fixed, OUT may stop (or
refuse to start) immediately even though the switch is physically released.

Usage: ros2 run ugv_tools gripper_repl
"""
import time
import subprocess
import Jetson.GPIO as GPIO
from Rosmaster_Lib import Rosmaster

# ============================================================
# CONFIGURATION
# ============================================================

CAR_TYPE = 4
PORTS = ['/dev/ttyUSB0', '/dev/myserial', '/dev/ttyACM0', '/dev/ttyTHS1']

# S1 continuous servo
S1_IN_SPEED = 50
S1_STOP = 90
S1_OUT_SPEED = 130

S1_IN_TIME = 20.0
S1_OUT_TIME = 5.0

# S2/S3
S2_UP = 180
S2_DOWN = 0

# Change this to adjust how quickly the S2/S3 command is sent.
# NOTE: PWM servos themselves do not expose a speed parameter through
# set_pwm_servo(). This controls the rate at which intermediate angles
# are commanded.
S23_STEP_DELAY = 0.03

# Jetson GPIO BOARD numbering
RELAY_OUT_PIN = 29
RELAY_IN_PIN = 31
SWITCH_PIN = 33

# Relay pinmux fixes for this carrier board
PINMUX_FIXUPS = {
    29: ['busybox', 'devmem', '0x2430068', 'w', '0x8'],
    31: ['busybox', 'devmem', '0x2430070', 'w', '0x8'],
}

current_s23_angle = 0

# ============================================================
# ROSMASTER
# ============================================================


def connect_rosmaster():
    for port in PORTS:
        try:
            bot = Rosmaster(car_type=CAR_TYPE, com=port, debug=False)
            bot.create_receive_threading()
            time.sleep(0.5)
            print(f"[OK] Rosmaster connected: {port}")
            return bot
        except Exception as e:
            print(f"[--] {port}: {e}")

    raise RuntimeError("Could not connect to Rosmaster.")


# ============================================================
# SWITCH
# ============================================================


def switch_pressed():
    # HIGH = pressed
    return GPIO.input(SWITCH_PIN) == GPIO.HIGH


# ============================================================
# SERVO COMMANDS
# ============================================================


def s1_stop(bot):
    bot.set_pwm_servo(1, S1_STOP)
    print("[S1] STOP")


def s1_in(bot):
    print(f"[S1] IN: speed={S1_IN_SPEED}, max {S1_IN_TIME:.0f}s")

    bot.set_pwm_servo(1, S1_IN_SPEED)

    start = time.monotonic()

    while time.monotonic() - start < S1_IN_TIME:
        time.sleep(0.05)

    s1_stop(bot)
    print("[S1] IN complete")


def s1_out(bot):
    print(
        f"[S1] OUT: speed={S1_OUT_SPEED}, "
        f"max {S1_OUT_TIME:.0f}s, switch active"
    )

    # Safety check before starting
    if switch_pressed():
        print("[S1] SWITCH ALREADY PRESSED - OUT cancelled")
        s1_stop(bot)
        return

    bot.set_pwm_servo(1, S1_OUT_SPEED)

    start = time.monotonic()

    while time.monotonic() - start < S1_OUT_TIME:

        if switch_pressed():
            print("[S1] SWITCH PRESSED -> STOP")
            s1_stop(bot)
            return

        time.sleep(0.02)

    s1_stop(bot)
    print("[S1] OUT timeout -> STOP")


def move_s2_s3(bot, target):
    """
    Move S2 and S3 together from their remembered position.

    S2 is mirrored relative to S3, so S2 receives the inverted
    physical angle.
    """

    global current_s23_angle

    start = current_s23_angle

    if start == target:
        print(f"[S2/S3] Already at {target}°")
        return

    direction = 1 if target > start else -1

    print(f"[S2/S3] Moving {start}° -> {target}°")

    angle = start

    while angle != target:
        angle += direction

        # S2 mirrored
        s2_physical = 180 - angle

        # S3 normal
        s3_physical = angle

        bot.set_pwm_servo(2, s2_physical)
        bot.set_pwm_servo(3, s3_physical)

        time.sleep(S23_STEP_DELAY)

    current_s23_angle = target

    print(f"[S2/S3] Position = {target}°")


def s23_stop(bot):
    """
    PWM positional servos don't have a true 'stop' command like S1.
    Re-send their current positions so they remain where they are.
    """
    bot.set_pwm_servo(2, 180 - current_s23_angle)
    bot.set_pwm_servo(3, current_s23_angle)

    print(
        f"[S2/S3] STOP/HOLD at {current_s23_angle}°"
    )


# ============================================================
# RELAYS
# ============================================================


def relay_stop():
    GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
    GPIO.output(RELAY_IN_PIN, GPIO.LOW)
    print("[RELAY] STOP")


def relay_push():
    print("[RELAY] PUSH / OUT for 20 seconds")

    GPIO.output(RELAY_IN_PIN, GPIO.LOW)
    GPIO.output(RELAY_OUT_PIN, GPIO.HIGH)

    try:
        time.sleep(20)
    finally:
        relay_stop()

    print("[RELAY] PUSH complete")


def relay_pull():
    print("[RELAY] PULL / IN for 20 seconds")

    GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
    GPIO.output(RELAY_IN_PIN, GPIO.HIGH)

    try:
        time.sleep(20)
    finally:
        relay_stop()

    print("[RELAY] PULL complete")


# ============================================================
# ALL STOP
# ============================================================


def all_stop(bot):
    print("[SYSTEM] STOP")

    s1_stop(bot)
    s23_stop(bot)
    relay_stop()


# ============================================================
# MAIN
# ============================================================


def setup_gpio():
    # Required on this carrier board
    for fixup in PINMUX_FIXUPS.values():
        subprocess.run(fixup, check=False)

    GPIO.setmode(GPIO.BOARD)

    GPIO.setup(RELAY_OUT_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(RELAY_IN_PIN, GPIO.OUT, initial=GPIO.LOW)

    GPIO.setup(SWITCH_PIN, GPIO.IN)

    print("[GPIO] Relay OUT  : pin 29")
    print("[GPIO] Relay IN   : pin 31")
    print("[GPIO] S1 switch  : pin 33")
    print(
        f"[GPIO] Switch     : "
        f"{'PRESSED/HIGH' if switch_pressed() else 'RELEASED/LOW'}"
    )


def print_help():
    print("""
Commands:

  IN       S1 inward, speed 50, maximum 20 seconds
  OUT      S1 outward, speed 130, maximum 5 seconds
           Stops immediately if pin 33 becomes HIGH

  UP       S2 + S3 -> 180 degrees
  DOWN     S2 + S3 -> 0 degrees

  STOP     Stop S1 and hold S2/S3 at current position
           Also stops both relays

  PUSH     Relay OUT for 20 seconds
  PULL     Relay IN for 20 seconds
  RSTOP    Stop both relays

  STATUS   Show switch and S2/S3 state
  HELP     Show this help
  QUIT     Stop everything and exit
""")


def main():
    global current_s23_angle

    bot = None

    try:
        setup_gpio()
        bot = connect_rosmaster()

        # Initial state
        s1_stop(bot)
        relay_stop()

        # Assumed startup position of S2/S3
        current_s23_angle = 0

        print("\n========================================")
        print("        GRIPPER HARDWARE CONTROL")
        print("========================================")
        print_help()

        while True:
            try:
                command = input("\nGRIPPER> ").strip().upper()
            except EOFError:
                break

            if not command:
                continue

            if command == "IN":
                s1_in(bot)

            elif command == "OUT":
                s1_out(bot)

            elif command == "UP":
                move_s2_s3(bot, 180)

            elif command == "DOWN":
                move_s2_s3(bot, 0)

            elif command == "STOP":
                all_stop(bot)

            elif command == "PUSH":
                relay_push()

            elif command == "PULL":
                relay_pull()

            elif command == "RSTOP":
                relay_stop()

            elif command == "STATUS":
                switch = "PRESSED/HIGH" if switch_pressed() else "RELEASED/LOW"

                print(f"[STATUS] Switch : {switch}")
                print(f"[STATUS] S2/S3  : {current_s23_angle}°")

            elif command == "HELP":
                print_help()

            elif command in ("QUIT", "EXIT"):
                break

            else:
                print(f"[ERROR] Unknown command: {command}")
                print("Type HELP for available commands.")

    except KeyboardInterrupt:
        print("\n[CTRL+C] Emergency stop")

    finally:
        if bot is not None:
            try:
                all_stop(bot)
            except Exception as e:
                print(f"[WARN] Could not stop servos: {e}")

        GPIO.cleanup()
        print("[SYSTEM] GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
