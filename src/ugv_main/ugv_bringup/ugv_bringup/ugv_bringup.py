import serial
import json
import queue
import threading
import rclpy
from rclpy.node import Node
import logging
import time
from std_msgs.msg import Header, Float32MultiArray, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField
import math
import os

def is_jetson():
    result = any("ugv_jetson" in root for root, dirs, files in os.walk("/"))
    return result

if is_jetson():
    serial_port = '/dev/ttyTHS1'
else:
    serial_port = '/dev/ttyAMA0'

# Helper class for reading lines from a serial port
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()  # Buffer to store incoming data
        self.s = s  # Serial object

    # Read a line of data from the serial input
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(512, self.s.in_waiting))  # Read from serial buffer
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

    # Clear the buffer
    def clear_buffer(self):
        self.s.reset_input_buffer()

# Base controller class for managing UART communication and processing commands
class BaseController:
    def __init__(self, uart_dev_set, baud_set):
        self.logger = logging.getLogger('BaseController')  # Logger setup
        self.ser = serial.Serial(uart_dev_set, baud_set, timeout=1)  # Open serial connection
        self.rl = ReadLine(self.ser)  # Initialize ReadLine helper
        self.command_queue = queue.Queue()  # Command queue for sending data
        self.command_thread = threading.Thread(target=self.process_commands, daemon=True)  # Start a separate thread for processing commands
        self.command_thread.start()
        self.data_buffer = None  # Buffer for holding received data
        # Base data structure to hold sensor values
        self.base_data = {"T": 1001, "L": 0, "R": 0, "ax": 0, "ay": 0, "az": 0, "gx": 0, "gy": 0, "gz": 0, "mx": 0, "my": 0, "mz": 0, "odl": 0, "odr": 0, "v": 0}
    
    # Function to read and return feedback data from the serial input
    def feedback_data(self):
        try:
            line = self.rl.readline().decode('utf-8')  # Read line from UART
            self.data_buffer = json.loads(line)  # Parse JSON data
            self.base_data = self.data_buffer  # Store received data
            return self.base_data  # Return base data
        except json.JSONDecodeError as e:
            self.logger.error(f"JSON decode error: {e} with line: {line}")  # Log error
            self.rl.clear_buffer()  # Clear buffer on error
        except Exception as e:
            self.logger.error(f"[base_ctrl.feedback_data] unexpected error: {e}")
            self.rl.clear_buffer()

    # Receive and decode data from the serial connection
    def on_data_received(self):
        self.ser.reset_input_buffer()
        data_read = json.loads(self.rl.readline().decode('utf-8'))  # Read and parse JSON data
        return data_read

    # Add a command to the queue to be sent via UART
    def send_command(self, data):
        self.command_queue.put(data)

    # Thread function to process and send commands from the queue
    def process_commands(self):
        while True:
            data = self.command_queue.get()  # Get command from the queue
            self.ser.write((json.dumps(data) + '\n').encode("utf-8"))  # Send command as JSON over UART

    # Send control data as JSON via UART
    def base_json_ctrl(self, input_json):
        self.send_command(input_json)

# ROS node class for bringing up the UGV system and publishing sensor data
class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        # Publishers for IMU data, magnetic field data, odometry, and voltage
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)
        # Initialize the base controller with the UART port and baud rate
        self.base_controller = BaseController(serial_port, 115200)

        # Gyro bias calibration: the raw gyro has a non-negligible, seemingly
        # variable zero-rate offset (empirically measured as ~8-34 deg/min of
        # stationary yaw drift downstream). do_bias_estimation in the
        # complementary filter doesn't fully remove it, so we additionally
        # measure our own static offset here at startup (robot MUST be
        # stationary for the first couple seconds after this node starts)
        # and subtract it from every subsequent raw reading before it's ever
        # published - this at least removes whatever the CURRENT session's
        # offset happens to be, even though it may still vary run-to-run.
        self.gyro_bias_samples = {"gx": [], "gy": [], "gz": []}
        self.gyro_bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}
        self.gyro_calibrated = False
        # Increased 200 -> 400 -> 2000 -> 5000: more samples reduce the noise
        # in the averaged bias estimate (roughly by 1/sqrt(N)), at the cost
        # of a slightly longer stationary warmup at startup. Empirically:
        #   200-400 samples:  ~8-34 deg/min stationary yaw drift
        #   2000 samples:     ~1.2 deg/min (~10 deg over 8.5 min)
        #   5000 samples:     ~0.5 deg/min (~5 deg over 10 min)
        self.GYRO_CALIBRATION_SAMPLES = 5000
        self.get_logger().info(
            f"Calibrating gyro bias ({self.GYRO_CALIBRATION_SAMPLES} samples) - keep the robot completely stationary..."
        )

        # Timer to periodically execute the feedback loop
        self.feedback_timer = self.create_timer(0.001, self.feedback_loop)

    # Main loop for reading sensor feedback and publishing it to ROS topics
    def feedback_loop(self):
        result = self.base_controller.feedback_data()
        # Only publish if a new, valid reading was actually received this cycle.
        # Otherwise we would republish stale data with a fresh timestamp, which
        # gets fed to the IMU filter as if it were a brand new measurement.
        if result is not None and self.base_controller.base_data["T"] == 1001:
            self.publish_imu_data_raw()  # Publish IMU raw data
            self.publish_imu_mag()  # Publish magnetic field data
            self.publish_odom_raw()  # Publish odometry data
            self.publish_voltage()  # Publish voltage data

    # Publish IMU data to the ROS topic "imu/data_raw"
    def publish_imu_data_raw(self):
        imu_raw_data = self.base_controller.base_data

        if not self.gyro_calibrated:
            self.gyro_bias_samples["gx"].append(float(imu_raw_data["gx"]))
            self.gyro_bias_samples["gy"].append(float(imu_raw_data["gy"]))
            self.gyro_bias_samples["gz"].append(float(imu_raw_data["gz"]))
            if len(self.gyro_bias_samples["gz"]) >= self.GYRO_CALIBRATION_SAMPLES:
                for axis in ("gx", "gy", "gz"):
                    samples = self.gyro_bias_samples[axis]
                    self.gyro_bias[axis] = sum(samples) / len(samples)
                self.gyro_calibrated = True
                self.get_logger().info(
                    f"Gyro bias calibration done: gx={self.gyro_bias['gx']:.2f} "
                    f"gy={self.gyro_bias['gy']:.2f} gz={self.gyro_bias['gz']:.2f} (raw units)"
                )
            # Don't publish anything until calibration completes - the raw
            # values aren't debiased yet, and publishing zeros/uncorrected
            # data would just feed bogus measurements into the IMU filter.
            return

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"

        # Populate the linear acceleration and angular velocity fields
        msg.linear_acceleration.x = 9.8 * float(imu_raw_data["ax"]) / 8192
        msg.linear_acceleration.y = 9.8 * float(imu_raw_data["ay"]) / 8192
        msg.linear_acceleration.z = 9.8 * float(imu_raw_data["az"]) / 8192

        gx = float(imu_raw_data["gx"]) - self.gyro_bias["gx"]
        gy = float(imu_raw_data["gy"]) - self.gyro_bias["gy"]
        gz = float(imu_raw_data["gz"]) - self.gyro_bias["gz"]
        msg.angular_velocity.x = 3.1415926 * gx / (16.4 * 180)
        msg.angular_velocity.y = 3.1415926 * gy / (16.4 * 180)
        msg.angular_velocity.z = 3.1415926 * gz / (16.4 * 180)

        self.imu_data_raw_publisher_.publish(msg)  # Publish the IMU data
        
    # Publish magnetic field data to the ROS topic "imu/mag"
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_imu_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the magnetic field data
        msg.magnetic_field.x = float(imu_raw_data["mx"]) * 0.15
        msg.magnetic_field.y = float(imu_raw_data["my"]) * 0.15
        msg.magnetic_field.z = float(imu_raw_data["mz"]) * 0.15
              
        self.imu_mag_publisher_.publish(msg)  # Publish the magnetic field data

    # Publish odometry data to the ROS topic "odom/odom_raw"
    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        array = [odom_raw_data["odl"]/100, odom_raw_data["odr"]/100]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)  # Publish the odometry data

    # Publish voltage data to the ROS topic "voltage"
    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        msg = Float32()
        msg.data = float(voltage_data["v"])/100
        self.voltage_publisher_.publish(msg)  # Publish the voltage data
                        
# Main function to initialize the ROS node and start spinning
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS
    node = ugv_bringup()  # Create the UGV bringup node
    rclpy.spin(node)  # Keep the node running
    #node.destroy_node()  # (optional) Shutdown the node
    rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    main()