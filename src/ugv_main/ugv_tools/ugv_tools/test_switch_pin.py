#!/usr/bin/env python3
"""Minimal standalone switch test - just prints the RAW level of one GPIO
pin, no active_low/active_high guessing, to isolate switch/wiring issues
from test_gripper_hw.py's more complex s1/relay logic. Defaults to pin 33
(suspected to be contended by the Jetson's fan PWM controller - see
/memories/repo/gripper_hw_investigation.md).

Usage: python3 test_switch_pin.py [duration_s] [pin]
"""
import sys
import time

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("Jetson.GPIO not importable - install it first (pip install Jetson.GPIO)")
    sys.exit(1)

DEFAULT_PIN = 33


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    pin = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PIN

    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(pin, GPIO.IN)
    print(f"Polling pin {pin} for {duration}s (Ctrl+C to stop early)...")
    start = time.monotonic()
    last = None
    try:
        while time.monotonic() - start < duration:
            level = GPIO.input(pin)
            if level != last:
                t = time.monotonic() - start
                state = "megvan nyomva" if level == GPIO.HIGH else "nincs megnyomva"
                print(f"  t={t:6.2f}s  level={'HIGH' if level == GPIO.HIGH else 'LOW'}  -> {state}")
                last = level
            time.sleep(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
