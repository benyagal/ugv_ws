#!/usr/bin/env python3
"""Gripper arm control node - thin USB-serial bridge to an Arduino running
the Jetson_to_Arduino.ino firmware (servos + relay-driven linear actuator +
homing switch + arm interpolation are all handled on the Arduino itself;
this node only forwards 'gripper_cmd' strings over serial and republishes
whatever the Arduino prints back as 'gripper_state').

Why this exists alongside gripper_node.py: gripper_node.py drives the
servos directly from the Jetson via a PCA9685 I2C PWM board. This node is
an alternative hardware path - a real Arduino handles all servo/relay/
homing timing in its own firmware, and the Jetson just relays commands
over USB-serial. gripper_node.py is left untouched/still usable; this is a
separate node + launch file so either hardware path can be run
independently.

Hardware:
  - Arduino (Uno-compatible) running Jetson_to_Arduino.ino, connected via
    USB. On this Jetson the board uses a CH340 USB-serial clone chip,
    which needed a custom-built 'ch341' kernel module (not shipped with
    this carrier board's stock kernel) - see repo notes/commit history for
    how that module was built and installed. Once loaded, the board shows
    up as /dev/ttyUSB0 (see the 'serial_port' parameter below).

Command/topic interface (same topic names as gripper_node.py, so existing
joystick/web-app publishers work unchanged):
  'gripper_cmd' (std_msgs/String): in, out, stop, home, up, down, push,
    pull, mstop, status, rollspeed <0-100>, inspeed <0-100>,
    outspeed <0-100>, armspeed <0-100>
  'gripper_state' (std_msgs/String): raw lines printed by the Arduino
    firmware over serial. NOTE: the Arduino firmware only prints for
    STOP/homing-complete/speed-change/STATUS/unknown-command - unlike
    gripper_node.py (which logs a line for every routine command purely
    for operator feedback), this node does not invent extra messages, it
    only relays what the Arduino actually sends.

The 'rollspeed' prefix (this project's ROS command convention) is
translated to the Arduino firmware's 'SPEED' command name; in/out/stop/
home/up/down/push/pull/mstop/status/inspeed/outspeed/armspeed pass
straight through unchanged (the Arduino uppercases whatever it receives
itself, so case does not matter here).
"""
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import serial


class GripperArduinoNode(Node):
    def __init__(self):
        super().__init__('gripper_arduino_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1.0)
        except serial.SerialException as exc:
            self.get_logger().error(
                f"Could not open serial port '{self.serial_port}' at "
                f'{self.baud_rate} baud ({exc}). Check that: the Arduino '
                f'is plugged in; the ch341 kernel module is loaded '
                f"(`lsmod | grep ch341` - reload with `sudo modprobe "
                f"ch341` if missing, it does not survive a reboot unless "
                f"added to /etc/modules); the device node exists "
                f"(`ls {self.serial_port}`); and this user is in the "
                "'dialout' group (`groups $USER`)."
            )
            raise

        # Most Arduino boards reset when the serial port is opened (DTR
        # toggling) - give it time to finish setup() and start loop()
        # before sending any commands, otherwise the first command(s) sent
        # immediately after startup can be lost while the bootloader/reset
        # is still in progress.
        time.sleep(2.0)

        self.create_subscription(String, 'gripper_cmd', self.cmd_callback, 10)
        self.state_pub = self.create_publisher(String, 'gripper_state', 10)

        self._stop_reader = threading.Event()
        self._reader_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._reader_thread.start()

        self.get_logger().info(
            f"Gripper Arduino bridge ready on '{self.serial_port}' at "
            f'{self.baud_rate} baud.'
        )

    def _read_loop(self):
        while not self._stop_reader.is_set():
            try:
                line = self.ser.readline()
            except serial.SerialException as exc:
                self.get_logger().error(f'Serial read error: {exc}')
                return
            if not line:
                continue
            text = line.decode('utf-8', errors='replace').strip()
            if not text:
                continue
            self.get_logger().info(text)
            self.state_pub.publish(String(data=text))

    def cmd_callback(self, msg: String):
        command = msg.data.strip().lower()
        if not command:
            return
        # This project's ROS convention uses 'rollspeed' where the Arduino
        # firmware expects 'SPEED' (sets both IN and OUT speed together) -
        # everything else lines up 1:1 already.
        if command.startswith('rollspeed'):
            command = 'speed' + command[len('rollspeed'):]
        try:
            self.ser.write((command + '\n').encode('utf-8'))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write error: {exc}')

    def destroy_node(self):
        self._stop_reader.set()
        try:
            self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperArduinoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
