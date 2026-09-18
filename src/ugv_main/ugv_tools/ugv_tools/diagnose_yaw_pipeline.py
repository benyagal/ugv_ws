#!/usr/bin/env python3
"""Bisect WHERE cumulative yaw error is introduced in the localization chain.

ROS2 node - run with `ros2 run ugv_tools diagnose_yaw_pipeline` while the
normal localization bringup (ugv_bringup + complementary_filter + ekf_local +
ekf_global) is already running.

calibrate_gyro_yaw.py already confirmed the raw, corrected IMU gz reading
(GYRO_Z_SCALE_CORRECTION applied in ugv_bringup.py) tracks a real rotation to
within ~1-4%. If a real rotation still shows a much larger error further
downstream (e.g. 270 real vs 450 reported), the extra error must be
introduced by one of: complementary_filter_node (/imu/data), ekf_local
(/odometry/local), or ekf_global (/odometry/global). This node watches all
three plus its own independent integration of /imu/data_raw side by side, so
a single physical rotation test shows exactly which stage's number departs
from the others.

Usage: start it, mark the robot's current heading, rotate it a known amount
(e.g. exactly 360 degrees), stop, and compare the four printed deltas -
whichever one departs from "raw_integrated" (and from the real rotation you
just did) is where the bug is.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

PRINT_PERIOD = 1.0  # seconds

# Below this |angular.z| a cmd_vel counts as "commanded stopped" for the
# moving/idle split of the raw gyro integral.
CMD_STOPPED_EPS = 1e-3


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def shortest_angle_diff(a, b):
    """Signed b-a difference wrapped to (-pi, pi] - for accumulating a
    continuous (unwrapped) angle across a quaternion's +-180 deg boundary."""
    d = (b - a + math.pi) % (2.0 * math.pi) - math.pi
    return d


class YawPipelineProbe(Node):
    def __init__(self):
        super().__init__('diagnose_yaw_pipeline')

        # Independent baseline: integrate /imu/data_raw's already-corrected
        # gz ourselves, using real message-to-message dt (not this node's own
        # timer), so it's not affected by anything complementary_filter or
        # the EKFs do.
        self._raw_integrated = 0.0
        self._raw_last_stamp = None

        # Split of _raw_integrated by what cmd_vel was commanding at the
        # time: yaw accumulated while commanded to rotate vs. while commanded
        # to hold still. A large "idle" share means the extra rotation is
        # either real coast-down after set_motor(0) (which freewheels rather
        # than brakes) or spurious gyro output at rest - not a scale error.
        self._raw_moving = 0.0
        self._raw_idle = 0.0
        self._cmd_rotating = False

        # Each source's yaw comes from a quaternion, bounded to (-180,180] -
        # accumulate the shortest-path delta between consecutive samples so
        # multi-turn (>180 deg) rotations don't show a false jump/wrap.
        self._accum = {}
        self._last_yaw = {}
        self._latest = {'raw_integrated': 0.0, 'imu_data': None, 'ekf_local': None, 'ekf_global': None}

        self.create_subscription(Imu, 'imu/data_raw', self._on_imu_raw, 20)
        self.create_subscription(Imu, 'imu/data', self._on_imu_data, 20)
        self.create_subscription(Odometry, 'odometry/local', self._on_ekf_local, 20)
        self.create_subscription(Odometry, 'odometry/global', self._on_ekf_global, 20)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 20)

        self.create_timer(PRINT_PERIOD, self._print_status)
        self.get_logger().info(
            "Watching imu/data_raw, imu/data, odometry/local, odometry/global - "
            "mark a heading, rotate a known amount, compare the printed deltas."
        )

    def _on_cmd_vel(self, msg):
        self._cmd_rotating = abs(msg.angular.z) > CMD_STOPPED_EPS

    def _on_imu_raw(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._raw_last_stamp is not None:
            dt = stamp - self._raw_last_stamp
            if 0.0 < dt < 1.0:
                step = msg.angular_velocity.z * dt
                self._raw_integrated += step
                if self._cmd_rotating:
                    self._raw_moving += step
                else:
                    self._raw_idle += step
        self._raw_last_stamp = stamp
        self._latest['raw_integrated'] = self._raw_integrated

    def _record(self, key, yaw):
        prev = self._last_yaw.get(key)
        if prev is not None:
            self._accum[key] = self._accum.get(key, 0.0) + shortest_angle_diff(prev, yaw)
        else:
            self._accum[key] = 0.0
        self._last_yaw[key] = yaw
        self._latest[key] = self._accum[key]

    def _on_imu_data(self, msg):
        self._record('imu_data', yaw_from_quaternion(msg.orientation))

    def _on_ekf_local(self, msg):
        self._record('ekf_local', yaw_from_quaternion(msg.pose.pose.orientation))

    def _on_ekf_global(self, msg):
        self._record('ekf_global', yaw_from_quaternion(msg.pose.pose.orientation))

    def _print_status(self):
        def deg(v):
            return f"{math.degrees(v):7.1f}" if v is not None else "  (none)"

        self.get_logger().info(
            f"raw_integrated={deg(self._latest['raw_integrated'])}  "
            f"imu_data={deg(self._latest['imu_data'])}  "
            f"ekf_local={deg(self._latest['ekf_local'])}  "
            f"ekf_global={deg(self._latest['ekf_global'])}  "
            f"[raw split: cmd_moving={deg(self._raw_moving)} cmd_idle={deg(self._raw_idle)}]  "
            f"(deg since node start)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = YawPipelineProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
