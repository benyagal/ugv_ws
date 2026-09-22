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
import threading
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

# How long to jog S1 back IN once the microswitch trips during OUT, so the
# actuator doesn't sit pressed against the switch/hard stop.
S1_SWITCH_BUMP_TIME = 0.5

# S2/S3
S2_UP = 160
S2_DOWN = 0

# Change this to adjust how quickly the S2/S3 command is sent.
# NOTE: PWM servos themselves do not expose a speed parameter through
# set_pwm_servo(). This controls the rate at which intermediate angles
# are commanded.
S23_STEP_DELAY = 0.005

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

# Every long-running command (S1 IN/OUT, S2/S3 move, relay PUSH/PULL) runs on
# this thread so the main input() loop stays free to accept STOP mid-command
# instead of blocking until the command finishes on its own.
stop_event = threading.Event()
active_thread = None


def run_in_thread(func, *args):
    global active_thread
    if active_thread is not None and active_thread.is_alive():
        print("[WARN] Another command is still running - send STOP first")
        return
    stop_event.clear()
    active_thread = threading.Thread(target=func, args=args, daemon=True)
    active_thread.start()


def cancel_active(timeout=2.0):
    stop_event.set()
    if active_thread is not None and active_thread.is_alive():
        active_thread.join(timeout)


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


def s1_bump_in(bot, duration):
    bot.set_pwm_servo(1, S1_IN_SPEED)

    start = time.monotonic()
    while time.monotonic() - start < duration:
        if stop_event.is_set():
            break
        time.sleep(0.02)

    s1_stop(bot)


def s1_in(bot):
    print(f"[S1] IN: speed={S1_IN_SPEED}, max {S1_IN_TIME:.0f}s")

    bot.set_pwm_servo(1, S1_IN_SPEED)

    start = time.monotonic()

    while time.monotonic() - start < S1_IN_TIME:
        if stop_event.is_set():
            s1_stop(bot)
            print("[S1] IN cancelled")
            return
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
            print(f"[S1] Bumping IN for {S1_SWITCH_BUMP_TIME:.1f}s to relieve the switch")
            s1_bump_in(bot, S1_SWITCH_BUMP_TIME)
            return

        if stop_event.is_set():
            s1_stop(bot)
            print("[S1] OUT cancelled")
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
        if stop_event.is_set():
            current_s23_angle = angle
            print(f"[S2/S3] Cancelled at {angle}\u00b0")
            return

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
        start = time.monotonic()
        while time.monotonic() - start < 20:
            if stop_event.is_set():
                break
            time.sleep(0.05)
    finally:
        relay_stop()

    print("[RELAY] PUSH complete")


def relay_pull():
    print("[RELAY] PULL / IN for 20 seconds")

    GPIO.output(RELAY_OUT_PIN, GPIO.LOW)
    GPIO.output(RELAY_IN_PIN, GPIO.HIGH)

    try:
        start = time.monotonic()
        while time.monotonic() - start < 20:
            if stop_event.is_set():
                break
            time.sleep(0.05)
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
    print(f"""
Commands:

  IN       S1 inward, speed 50, maximum 20 seconds
  OUT      S1 outward, speed 130, maximum 5 seconds
           Stops immediately if pin 33 becomes HIGH

  UP       S2 + S3 -> {S2_UP} degrees
  DOWN     S2 + S3 -> {S2_DOWN} degrees

  STOP     Cancel whatever is running right now (IN/OUT/UP/DOWN/PUSH/PULL)
           and hold S2/S3 at current position; also stops both relays

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

        # Assumed startup position of S2/S3 - matches S2_DOWN since the
        # gripper physically rests there at power-on.
        current_s23_angle = S2_DOWN

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
                run_in_thread(s1_in, bot)

            elif command == "OUT":
                run_in_thread(s1_out, bot)

            elif command == "UP":
                run_in_thread(move_s2_s3, bot, S2_UP)

            elif command == "DOWN":
                run_in_thread(move_s2_s3, bot, S2_DOWN)

            elif command == "STOP":
                cancel_active()
                all_stop(bot)

            elif command == "PUSH":
                run_in_thread(relay_push)

            elif command == "PULL":
                run_in_thread(relay_pull)

            elif command == "RSTOP":
                cancel_active()
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
        cancel_active()
        if bot is not None:
            try:
                all_stop(bot)
            except Exception as e:
                print(f"[WARN] Could not stop servos: {e}")

        GPIO.cleanup()
        print("[SYSTEM] GPIO cleaned up. Exiting.")


if __name__ == "__main__":
    main()
