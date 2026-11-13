#!/usr/bin/env python3
"""Battery level and charging state on the SSD1306 OLED wired to the Jetson I2C header."""
import fcntl
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float32

I2C_SLAVE = 0x0703

# Bar thresholds from the 3S discharge curve (12.46 V full, firmware cuts motors at 9.6 V).
LEVEL_VOLTAGES = (9.7, 10.65, 11.35, 11.9)
SMOOTH_TAU = 10.0      # s, damps the sag while driving
# Without charging the level only drops; this much above a threshold still raises it
# (charger contact missed, e.g. on a nearly full battery).
LEVEL_RISE_MARGIN = 0.3  # V
REFRESH_PERIOD = 0.5   # s, also the charging animation step
REOPEN_PERIOD = 5.0    # s

# 5x7 glyphs, one byte per column, bit 0 = top row.
FONT = {
    ' ': (0x00, 0x00, 0x00, 0x00, 0x00),
    '-': (0x08, 0x08, 0x08, 0x08, 0x08),
    '.': (0x00, 0x60, 0x60, 0x00, 0x00),
    '0': (0x3E, 0x51, 0x49, 0x45, 0x3E),
    '1': (0x00, 0x42, 0x7F, 0x40, 0x00),
    '2': (0x42, 0x61, 0x51, 0x49, 0x46),
    '3': (0x21, 0x41, 0x45, 0x4B, 0x31),
    '4': (0x18, 0x14, 0x12, 0x7F, 0x10),
    '5': (0x27, 0x45, 0x45, 0x45, 0x39),
    '6': (0x3C, 0x4A, 0x49, 0x49, 0x30),
    '7': (0x01, 0x71, 0x09, 0x05, 0x03),
    '8': (0x36, 0x49, 0x49, 0x49, 0x36),
    '9': (0x06, 0x49, 0x49, 0x29, 0x1E),
    'L': (0x7F, 0x40, 0x40, 0x40, 0x40),
    'O': (0x3E, 0x41, 0x41, 0x41, 0x3E),
    'T': (0x01, 0x01, 0x7F, 0x01, 0x01),
    'V': (0x1F, 0x20, 0x40, 0x20, 0x1F),
    'W': (0x3F, 0x40, 0x38, 0x40, 0x3F),
    'é': (0x38, 0x54, 0x56, 0x55, 0x18),
    'l': (0x00, 0x41, 0x7F, 0x40, 0x00),
    'ö': (0x38, 0x45, 0x44, 0x45, 0x38),
    's': (0x48, 0x54, 0x54, 0x54, 0x20),
    't': (0x04, 0x3F, 0x44, 0x40, 0x20),
}


def battery_level(v):
    return sum(v >= t for t in LEVEL_VOLTAGES)


class Ssd1306:
    def __init__(self, bus, address, width, height, flip):
        self.width = width
        self.height = height
        self.fd = os.open(f'/dev/i2c-{bus}', os.O_RDWR)
        try:
            fcntl.ioctl(self.fd, I2C_SLAVE, address)
            self.command(
                0xAE, 0xD5, 0x80, 0xA8, height - 1, 0xD3, 0x00, 0x40,
                0x8D, 0x14, 0x20, 0x00,
                0xA0 if flip else 0xA1, 0xC0 if flip else 0xC8,
                0xDA, 0x02 if height == 32 else 0x12,
                0x81, 0x8F, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6, 0xAF)
        except OSError:
            self.close()
            raise

    def command(self, *cmds):
        os.write(self.fd, bytes((0x00,) + cmds))

    def show(self, buf):
        self.command(0x21, 0, self.width - 1, 0x22, 0, self.height // 8 - 1)
        for i in range(0, len(buf), 32):
            os.write(self.fd, b'\x40' + bytes(buf[i:i + 32]))

    def close(self):
        os.close(self.fd)


class Canvas:
    def __init__(self, width, height):
        self.width = width
        self.buf = bytearray(width * height // 8)

    def pixel(self, x, y):
        self.buf[(y // 8) * self.width + x] |= 1 << (y % 8)

    def rect(self, x0, y0, x1, y1, fill=True):
        for x in range(x0, x1 + 1):
            for y in range(y0, y1 + 1):
                if fill or x in (x0, x1) or y in (y0, y1):
                    self.pixel(x, y)

    def text(self, x, y, s, scale=2):
        for ch in s:
            for col, bits in enumerate(FONT.get(ch, FONT[' '])):
                for row in range(7):
                    if bits >> row & 1:
                        self.rect(x + col * scale, y + row * scale,
                                  x + (col + 1) * scale - 1, y + (row + 1) * scale - 1)
            x += 6 * scale


class OledDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')
        self.bus = self.declare_parameter('i2c_bus', 7).value
        self.address = self.declare_parameter('i2c_address', 0x3C).value
        self.width = 128
        self.height = self.declare_parameter('height', 32).value
        self.flip = self.declare_parameter('flip', False).value

        self.oled = None
        self._last_open_try = -REOPEN_PERIOD
        self._shown = None
        self.voltage = None
        self._last_voltage_time = None
        self.level = None
        self.charging = False
        self._step = 0

        self.create_subscription(Float32, 'voltage', self.voltage_callback, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, 'charging', self.charging_callback, latched)
        self.create_timer(REFRESH_PERIOD, self.refresh)

    def voltage_callback(self, msg):
        if msg.data <= 0.1:
            return
        now = time.monotonic()
        if self.voltage is None:
            self.voltage = msg.data
        else:
            alpha = min(1.0, (now - self._last_voltage_time) / SMOOTH_TAU)
            self.voltage += alpha * (msg.data - self.voltage)
        self._last_voltage_time = now

        level = battery_level(self.voltage)
        if self.level is None or self.charging:
            self.level = level
        else:
            self.level = max(min(self.level, level),
                             battery_level(self.voltage - LEVEL_RISE_MARGIN))

    def charging_callback(self, msg):
        self.charging = msg.data

    def draw(self):
        c = Canvas(self.width, self.height)
        top = (self.height - 32) // 2
        c.rect(0, top + 4, 37, top + 27, fill=False)
        c.rect(38, top + 11, 40, top + 20)

        bars = self.level or 0
        if self.charging:
            # Bars fill up from the current level; a full battery blinks its last bar.
            self._step += 1
            bars = 3 + self._step % 2 if bars >= 4 else bars + self._step % (5 - bars)
        for i in range(bars):
            x = 2 + i * 9
            c.rect(x, top + 6, x + 6, top + 25)

        c.text(46, top, '--.-V' if self.voltage is None else f'{self.voltage:4.1f}V')
        if self.charging:
            c.text(46, top + 18, 'Töltés')
        elif self.level == 0:
            c.text(46, top + 18, 'LOW')
        return c.buf

    def refresh(self):
        now = time.monotonic()
        if self.oled is None:
            if now - self._last_open_try < REOPEN_PERIOD:
                return
            self._last_open_try = now
            try:
                self.oled = Ssd1306(self.bus, self.address, self.width, self.height, self.flip)
                self._shown = None
                self.get_logger().info(
                    f'OLED on /dev/i2c-{self.bus} at 0x{self.address:02x}')
            except OSError as e:
                self.get_logger().warn(
                    f'OLED /dev/i2c-{self.bus} 0x{self.address:02x} unavailable: {e}',
                    throttle_duration_sec=60.0)
                return

        buf = self.draw()
        if buf == self._shown:
            return
        try:
            self.oled.show(buf)
            self._shown = buf
        except OSError as e:
            self.get_logger().warn(f'OLED write failed: {e}')
            self.oled.close()
            self.oled = None

    def shutdown(self):
        if self.oled is None:
            return
        try:
            self.oled.command(0xAE)
        except OSError:
            pass
        self.oled.close()


def main(args=None):
    rclpy.init(args=args)
    node = OledDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
