#!/usr/bin/env python3
"""One-off diagnostic: dump Jetson.GPIO's internal BOARD-pin -> Linux GPIO
chip/line table, filtered to the pins we care about (29, 31, 33), so the
exact `gpioget <chip> <line>` invocation for physical pin 33 can be derived
without guessing from `gpioinfo`'s pinmux names (PF.xx/PG.xx/...).

Run directly, no colcon build needed:
  python3 find_gpio_line.py
"""
import sys

try:
    import Jetson.GPIO as GPIO
except ImportError:
    print("Jetson.GPIO not importable - install it first (pip install Jetson.GPIO)")
    sys.exit(1)

TARGET_BOARD_PINS = {29, 31, 33}


def main():
    try:
        from Jetson.GPIO import gpio_pin_data
    except ImportError:
        print("Could not import Jetson.GPIO.gpio_pin_data - dumping GPIO module dir instead:")
        print([n for n in dir(GPIO) if not n.startswith('_')])
        return

    try:
        result = gpio_pin_data.get_data()
    except Exception as e:
        print(f"gpio_pin_data.get_data() failed: {e!r}")
        print("Falling back to raw module dir:")
        print([n for n in dir(gpio_pin_data) if not n.startswith('_')])
        return

    print(f"get_data() returned {len(result)} values")
    model = key = jetson_gpio_data = None
    if len(result) == 3:
        model, jetson_gpio_data, key = result
    else:
        print("Unexpected shape, printing raw:")
        print(result)
        return

    print(f"model={model!r}  key={key!r}")
    rows = jetson_gpio_data.get(key, []) if isinstance(jetson_gpio_data, dict) else []
    print(f"{len(rows)} total pin entries for this board; filtering for {TARGET_BOARD_PINS}...")
    for entry in rows:
        fields = entry._asdict() if hasattr(entry, '_asdict') else entry
        values = fields.values() if isinstance(fields, dict) else fields
        if any(str(v) in {str(p) for p in TARGET_BOARD_PINS} for v in values):
            print(fields)


if __name__ == '__main__':
    main()
