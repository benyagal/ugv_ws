#!/usr/bin/env python3
"""Minimal PCA9685 16-channel PWM driver over I2C (smbus2).

Register-level re-implementation of the calls the original Arduino sketch
used from Adafruit_PWMServoDriver (pwm.begin(), pwm.setPWMFreq(), pwm.setPWM()),
so no Adafruit/Blinka dependency is needed on the Jetson.
"""
import time

import smbus2

PCA9685_ADDRESS = 0x40

MODE1 = 0x00
MODE2 = 0x01
PRESCALE = 0xFE
LED0_ON_L = 0x06

MODE1_SLEEP = 0x10
MODE1_AUTOINC = 0x20
MODE1_RESTART = 0x80


class PCA9685:
    # The Jetson's I2C bus occasionally reports transient errors (e.g.
    # "arbitration lost", seen in dmesg as tegra-i2c ... un-recovered
    # arbitration lost), most likely from electrical noise from the
    # nearby relay/motor switching. A short retry usually succeeds on
    # the next attempt, so we retry a few times before giving up.
    I2C_RETRY_ATTEMPTS = 3
    I2C_RETRY_DELAY = 0.01

    def __init__(self, bus_num: int, address: int = PCA9685_ADDRESS):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self._write8(MODE1, 0x00)
        time.sleep(0.005)

    def _retry(self, func, *args):
        last_error = None
        for attempt in range(self.I2C_RETRY_ATTEMPTS):
            try:
                return func(*args)
            except OSError as e:
                last_error = e
                if attempt < self.I2C_RETRY_ATTEMPTS - 1:
                    time.sleep(self.I2C_RETRY_DELAY)
        raise last_error

    def _write8(self, reg: int, value: int):
        self._retry(self.bus.write_byte_data, self.address, reg, value & 0xFF)

    def _read8(self, reg: int) -> int:
        return self._retry(self.bus.read_byte_data, self.address, reg)

    def set_pwm_freq(self, freq_hz: float):
        prescale_val = int(round(25000000.0 / (4096 * freq_hz)) - 1)
        old_mode = self._read8(MODE1)
        sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP
        self._write8(MODE1, sleep_mode)
        self._write8(PRESCALE, prescale_val)
        self._write8(MODE1, old_mode)
        time.sleep(0.005)
        self._write8(MODE1, old_mode | MODE1_RESTART | MODE1_AUTOINC)

    def set_pwm(self, channel: int, on: int, off: int):
        base = LED0_ON_L + 4 * channel
        self._write8(base, on & 0xFF)
        self._write8(base + 1, (on >> 8) & 0xFF)
        self._write8(base + 2, off & 0xFF)
        self._write8(base + 3, (off >> 8) & 0xFF)

    def close(self):
        self.bus.close()
