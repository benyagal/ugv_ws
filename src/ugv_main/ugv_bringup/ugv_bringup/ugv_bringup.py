import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray, Float32
from sensor_msgs.msg import Imu, MagneticField

from Rosmaster_Lib import Rosmaster

# Yahboom Rosmaster (STM32F103RCT6) board, replacing the original (broken)
# motor controller. CAR_TYPE=4 (CAR_FOURWHEEL) confirmed from the firmware
# source (app_motion.h) as the plain differential/skid-steer kinematics that
# matches our robot's wheelbase-based drive - NOT mecanum(1) or Ackermann(5).
# See /memories/repo/rosmaster_motor_controller.md for the full investigation.
CAR_TYPE = 4

# Confirmed via `ls -l /dev/ttyUSB*` on the Jetson (2026-09-11) - the board has
# no udev rule yet, so this is the raw USB-serial enumeration, NOT guaranteed
# to stay ttyUSB0 if other USB-serial devices are plugged in a different order.
# TODO: add a udev rule (see ldlidar.rules for the pattern) for a stable symlink.
SERIAL_PORT = '/dev/ttyUSB0'

# Wheel-distance-per-encoder-pulse scale. The nominal numbers come from
# Yahboom's OWN 330RPM motor spec (app_motion.h: DISTANCE_CIRCLE=0.204203m /
# ENCODER_CIRCLE_330=1320 pulses/rev), which do not match our motors: the
# firmware's get_motion_data() speed, computed from these same constants, was
# measured to overreport real speed by 1/0.82 (see LINEAR_TELEMETRY_CORRECTION
# in ugv_driver.py), so the pulse scale carries the same error.
WHEEL_CIRCUMFERENCE_M = 0.204203
ENCODER_PULSES_PER_REV = 1320.0
ENCODER_SCALE_CORRECTION = 0.82
METERS_PER_PULSE = WHEEL_CIRCUMFERENCE_M / ENCODER_PULSES_PER_REV * ENCODER_SCALE_CORRECTION


class UgvBringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 100)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 100)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 100)
        self.voltage_publisher_ = self.create_publisher(Float32, "voltage", 50)

        self.car = Rosmaster(car_type=CAR_TYPE, com=SERIAL_PORT)
        self.car.create_receive_threading()
        self.car.set_auto_report_state(True, forever=False)

        # Cumulative left/right wheel distance (meters) - same contract as
        # the old board's odl/odr, so base_node.cpp doesn't need to change.
        # CAR_FOURWHEEL encoder order is (L1, L2, R1, R2) - see app_fourwheel.c.
        self._left_m = 0.0
        self._right_m = 0.0
        self._last_encoder = None

        # Gyro bias calibration: same rationale as the old board (measured
        # ~8-34 deg/min stationary yaw drift there). Rosmaster_Lib already
        # returns physical units (rad/s), so only a static offset subtraction
        # is needed here, no unit conversion.
        self.gyro_bias_samples = {"gx": [], "gy": [], "gz": []}
        self.gyro_bias = {"gx": 0.0, "gy": 0.0, "gz": 0.0}
        self.gyro_calibrated = False
        self.GYRO_CALIBRATION_SAMPLES = 5000

        # Yaw-rate scale correction (2026-09-17) - the new board's IMU chip
        # may use a different raw-to-rad/s conversion than Rosmaster_Lib
        # assumes (see the MPU9250 vs ICM20948 branches in its __parse_data,
        # which use different gyro_ratio constants) - a scale mismatch here
        # accumulates into visible yaw drift over repeated turns even though
        # the (stationary) bias calibration above is correct. Measured
        # 2026-09-18 via ugv_tools/calibrate_gyro_yaw.py (weighted fit across
        # 4 runs of ~115-516 deg, ratio range 1.011-1.036).
        GYRO_Z_SCALE_CORRECTION = 1.015
        self.gyro_z_scale_correction = GYRO_Z_SCALE_CORRECTION
        self.get_logger().info(
            f"Calibrating gyro bias ({self.GYRO_CALIBRATION_SAMPLES} samples) - keep the robot completely stationary..."
        )

        # Rosmaster's MCU auto-reports data every 40ms (AUTO_SEND_TIMEOUT) -
        # match that instead of the old board's much faster 1ms poll.
        self.feedback_timer = self.create_timer(0.04, self.feedback_loop)

    def feedback_loop(self):
        self.publish_imu_data_raw()
        self.publish_imu_mag()
        self.publish_odom_raw()
        self.publish_voltage()

    def publish_imu_data_raw(self):
        gx, gy, gz = self.car.get_gyroscope_data()

        if not self.gyro_calibrated:
            self.gyro_bias_samples["gx"].append(gx)
            self.gyro_bias_samples["gy"].append(gy)
            self.gyro_bias_samples["gz"].append(gz)
            if len(self.gyro_bias_samples["gz"]) >= self.GYRO_CALIBRATION_SAMPLES:
                for axis in ("gx", "gy", "gz"):
                    samples = self.gyro_bias_samples[axis]
                    self.gyro_bias[axis] = sum(samples) / len(samples)
                self.gyro_calibrated = True
                self.get_logger().info(
                    f"Gyro bias calibration done: gx={self.gyro_bias['gx']:.4f} "
                    f"gy={self.gyro_bias['gy']:.4f} gz={self.gyro_bias['gz']:.4f} (rad/s)"
                )
            # Don't publish until calibration completes - see old-code rationale.
            return

        ax, ay, az = self.car.get_accelerometer_data()

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az
        msg.angular_velocity.x = gx - self.gyro_bias["gx"]
        msg.angular_velocity.y = gy - self.gyro_bias["gy"]
        msg.angular_velocity.z = (gz - self.gyro_bias["gz"]) * self.gyro_z_scale_correction
        self.imu_data_raw_publisher_.publish(msg)

    def publish_imu_mag(self):
        mx, my, mz = self.car.get_magnetometer_data()
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_imu_link"
        msg.magnetic_field.x = mx
        msg.magnetic_field.y = my
        msg.magnetic_field.z = mz
        self.imu_mag_publisher_.publish(msg)

    def publish_odom_raw(self):
        encoder = self.car.get_motor_encoder()
        if self._last_encoder is not None:
            self._left_m += (encoder[0] - self._last_encoder[0]) * METERS_PER_PULSE
            self._right_m += (encoder[2] - self._last_encoder[2]) * METERS_PER_PULSE
        self._last_encoder = encoder

        msg = Float32MultiArray(data=[self._left_m, self._right_m])
        self.odom_publisher_.publish(msg)

    def publish_voltage(self):
        msg = Float32()
        msg.data = self.car.get_battery_voltage()
        self.voltage_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UgvBringup()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()