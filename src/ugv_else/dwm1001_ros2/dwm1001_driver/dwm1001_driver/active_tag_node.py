# Copyright 2023 The Human and Intelligent Vehicle Ensembles (HIVE) Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from collections import deque
import time

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from geometry_msgs.msg import PointStamped, TransformStamped

import serial

# The tag needs well over the dwm1001 lib's 0.1 s to reboot; shorter waits lose the
# shell-entry keystrokes and leave the tag silently in TLV mode.
RESET_BOOT_DELAY_S = 2.0
SHELL_STARTUP_DELAY_S = 1.0
SHELL_COMMAND_DELAY_S = 0.5
NO_DATA_TIMEOUT_S = 3.0
MAX_RX_BUFFER_BYTES = 1024


class ActiveTagNode(Node):
    def __init__(self) -> None:
        super().__init__("dwm_active", allow_undeclared_parameters=True)

        self._declare_parameters()

        samples_param = self.get_parameter("samples").value
        if samples_param > 10:
            self.get_logger().warn("Number of samples exceeds maximum value. Capping at 10.")
            samples_param = 10
        self.get_logger().info(f"Performing moving average with {samples_param} samples.")
        self._position_buffer = deque(maxlen=samples_param)

        serial_port_param = self.get_parameter("serial_port").value
        self.get_logger().info(f"Provided serial port: '{serial_port_param}'")

        self._serial = self._open_serial_port(serial_port_param)
        self._rx_buffer = b""
        self._start_position_reporting()

        tag_topic = f"~/output/{self.get_parameter('tag_id').value}"
        self.point_publisher = self.create_publisher(PointStamped, tag_topic, 1)
        self.get_logger().info(f"Publishing DWM postions on {tag_topic}")
        self.timer = self.create_timer(1 / 25, self.timer_callback)

    def _open_serial_port(self, serial_port: str) -> serial.Serial:
        if not serial_port:
            self._shutdown_fatal("No serial port specified.")

        try:
            serial_handle = serial.Serial(serial_port, baudrate=115_200, timeout=0.2)
        except serial.SerialException:
            self._shutdown_fatal(f"Could not open serial port '{serial_port}'.")

        self.get_logger().info(f"Opened serial port: '{serial_port}'.")

        return serial_handle

    def _send(self, data: bytes, delay_s: float) -> None:
        self._serial.write(data)
        self._serial.flush()
        time.sleep(delay_s)

    def _start_position_reporting(self) -> None:
        """Bring the tag from any state (TLV, idle shell, running lep) to shell + lep."""
        try:
            self._send(b"reset\r", RESET_BOOT_DELAY_S)
            self._send(b"\r\r", SHELL_STARTUP_DELAY_S)
            self._serial.reset_input_buffer()
            self._send(b"lep\r", SHELL_COMMAND_DELAY_S)
            # First line after lep has the shell prompt mixed in.
            self._serial.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error while starting position reporting: {e}")
        self._rx_buffer = b""
        self._last_report_time = time.monotonic()
        self.get_logger().info("Started position reporting.")

    def stop_position_reporting(self) -> None:
        """Stop lep and return to TLV mode so the next start begins from a known state."""
        try:
            self._send(b"\r", SHELL_COMMAND_DELAY_S)
            self._send(b"quit\r", SHELL_COMMAND_DELAY_S)
            self._serial.close()
        except serial.SerialException:
            pass

    def _parse_position(self, line: bytes):
        fields = line.decode(errors="ignore").strip().split(",")
        if fields[0] != "POS":
            return None
        try:
            if len(fields) != 5:
                raise ValueError(f"unexpected length {len(fields)}")
            return float(fields[1]), float(fields[2]), float(fields[3])
        except ValueError as e:
            self.get_logger().warn(f"Could not parse position report ({e}). Skipping it.")
            return None

    def _shutdown_fatal(self, message: str) -> None:
        self.get_logger().fatal(message + " Shutting down.")
        exit()

    def _declare_parameters(self):
        
        serial_port_descriptor = ParameterDescriptor(
            description="Device file or COM port associated with DWM1001",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )
        
        tag_id_descriptor = ParameterDescriptor(
            description="The ID for the particular DWM1001 device.",
            type=ParameterType.PARAMETER_STRING,
            read_only=True,
        )

        samples_descriptor = ParameterDescriptor(
            description="The number of samples used to perform the moving average. Max = 10",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=True,
        )
        self.declare_parameter("samples", 3, samples_descriptor)
        self.declare_parameter("tag_id", "", tag_id_descriptor)
        self.declare_parameter("serial_port", "", serial_port_descriptor)


    def timer_callback(self):
        try:
            waiting = self._serial.in_waiting
            if waiting:
                self._rx_buffer += self._serial.read(waiting)
        except serial.SerialException as e:
            self.get_logger().warn(f"Transient serial read error, skipping this cycle: {e}")

        *lines, self._rx_buffer = self._rx_buffer.split(b"\n")
        if len(self._rx_buffer) > MAX_RX_BUFFER_BYTES:
            self._rx_buffer = b""

        for line in lines:
            position = self._parse_position(line)
            if position is not None:
                self._last_report_time = time.monotonic()
                self._handle_position(position)

        if time.monotonic() - self._last_report_time > NO_DATA_TIMEOUT_S:
            self.get_logger().warn(
                f"No POS reports for {NO_DATA_TIMEOUT_S:.0f} s, reinitializing tag.")
            self._start_position_reporting()

    def _handle_position(self, position):
        self._position_buffer.append(position)

        # Nothing reported until the position buffer is filled. Then a moving average is computed.
        if len(self._position_buffer) == self._position_buffer.maxlen:

            x_avg = round(sum([point[0] for point in self._position_buffer]) / self._position_buffer.maxlen, 3)
            y_avg = round(sum([point[1] for point in self._position_buffer]) / self._position_buffer.maxlen, 3)
            z_avg = round(sum([point[2] for point in self._position_buffer]) / self._position_buffer.maxlen, 3)

            time_stamp = self.get_clock().now().to_msg()
            msg = PointStamped()

            msg.header.stamp = time_stamp
            msg.header.frame_id = "dwm1001"
            msg.point.x = x_avg
            msg.point.y = y_avg
            msg.point.z = z_avg

            self.point_publisher.publish(msg)
        else:
            self.get_logger().warn(f"DWM1001 buffer not filled to {self._position_buffer.maxlen} yet.")


def main(args=None):
    rclpy.init(args=args)

    active_tag = ActiveTagNode()
    try:
        rclpy.spin(active_tag)
    except KeyboardInterrupt:
        pass
    finally:
        active_tag.stop_position_reporting()
        active_tag.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
