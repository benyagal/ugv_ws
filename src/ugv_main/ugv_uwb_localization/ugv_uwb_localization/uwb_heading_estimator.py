#!/usr/bin/env python3
"""Map-frame heading from UWB positions recorded while the robot drives straight.

Publishes /uwb/heading (yaw-only PoseWithCovarianceStamped, fused by ekf_global
when uwb_heading_correction:=true) and /uwb/heading_calibration_request: 0 when
no calibration is needed, otherwise an id that increases on every retry. The
Nav2 HeadingCalibration BT node answers each new id with one straight drive.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import UInt32


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class UwbHeadingEstimator(Node):
    def __init__(self):
        super().__init__('uwb_heading_estimator')
        param = self.declare_parameter
        self.min_speed = param('min_speed', 0.08).value
        self.max_yaw_rate = param('max_yaw_rate', 0.05).value
        self.max_yaw_change = math.radians(param('max_yaw_change_deg', 3.0).value)
        self.min_length = param('min_segment_length', 1.0).value
        self.max_length = param('max_segment_length', 3.0).value
        self.min_samples = param('min_samples', 15).value
        self.max_rms = param('max_rms', 0.08).value
        self.max_residual = param('max_residual', 0.25).value
        self.min_length_ratio = param('min_length_ratio', 0.7).value
        self.max_length_ratio = param('max_length_ratio', 1.3).value
        # UWB errors are time-correlated, so N samples are worth fewer independent ones.
        self.variance_inflation = param('variance_inflation', 4.0).value
        self.min_sigma = math.radians(param('min_sigma_deg', 2.0).value)
        self.max_correction = math.radians(param('max_correction_deg', 45.0).value)
        self.active_requests = param('active_requests', False).value
        self.request_interval = param('request_interval', 120.0).value
        self.request_retry_interval = param('request_retry_interval', 60.0).value

        self.heading_pub = self.create_publisher(PoseWithCovarianceStamped, '/uwb/heading', 10)
        self.request_pub = self.create_publisher(UInt32, '/uwb/heading_calibration_request', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/uwb/pose', self.uwb_callback, 10)
        self.create_subscription(Odometry, '/odometry/local', self.local_callback, 20)
        self.create_subscription(Odometry, '/odometry/global', self.global_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10)
        self.create_timer(1.0, self.request_tick)

        self.active = False
        self.direction = 1.0
        self.points = []
        self.yaw_ref = 0.0
        self.yaw_offsets = []
        self.latest_yaw_offset = 0.0
        self.latest_stamp = None
        self.odom_distance = 0.0
        self.last_local_time = None
        self.global_yaw = None

        self.last_accepted = self.now_sec()
        self.last_request = -math.inf
        self.request_id = 0
        self.request_pending = False

        self.get_logger().info(
            f'UWB heading estimator started (active_requests={self.active_requests})')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def start_segment(self, direction, yaw, stamp):
        self.active = True
        self.direction = direction
        self.points = []
        self.yaw_ref = yaw
        self.yaw_offsets = [0.0]
        self.latest_yaw_offset = 0.0
        self.latest_stamp = stamp
        self.odom_distance = 0.0

    def end_segment(self, end_reason):
        self.active = False
        if self.odom_distance >= 0.5 * self.min_length:
            self.evaluate(end_reason)

    def local_callback(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        vx = msg.twist.twist.linear.x
        wz = msg.twist.twist.angular.z
        straight = abs(vx) >= self.min_speed and abs(wz) <= self.max_yaw_rate
        direction = math.copysign(1.0, vx)
        yaw = yaw_of(msg.pose.pose.orientation)

        if self.active:
            offset = wrap(yaw - self.yaw_ref)
            span = max(self.yaw_offsets + [offset]) - min(self.yaw_offsets + [offset])
            end_reason = None
            if abs(vx) < self.min_speed:
                end_reason = f'slow/stopped v={vx:.2f} m/s'
            elif abs(wz) > self.max_yaw_rate:
                end_reason = f'turning {wz:+.3f} rad/s > {self.max_yaw_rate:.3f}'
            elif direction != self.direction:
                end_reason = 'direction change'
            elif span > self.max_yaw_change:
                end_reason = (f'heading changed {math.degrees(span):.1f} deg > '
                              f'{math.degrees(self.max_yaw_change):.1f}')
            elif self.odom_distance >= self.max_length:
                end_reason = f'max length {self.max_length:.1f} m'
            if end_reason is not None:
                self.end_segment(end_reason)
            else:
                dt = min(max(stamp - self.last_local_time, 0.0), 0.5)
                self.odom_distance += abs(vx) * dt
                self.yaw_offsets.append(offset)
                self.latest_yaw_offset = offset
                self.latest_stamp = msg.header.stamp

        if straight and not self.active:
            self.start_segment(direction, yaw, msg.header.stamp)
        self.last_local_time = stamp

    def uwb_callback(self, msg):
        if self.active:
            self.points.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def global_callback(self, msg):
        self.global_yaw = yaw_of(msg.pose.pose.orientation)

    def initialpose_callback(self, _msg):
        self.active = False
        self.last_accepted = self.now_sec()
        self.request_pending = False

    def evaluate(self, end_reason):
        n = len(self.points)
        if n < 3:
            self.get_logger().info(
                f'segment {self.odom_distance:.2f} m (odom) -> rejected: only {n} UWB samples '
                f'[end: {end_reason}]')
            return

        mx = sum(x for x, _ in self.points) / n
        my = sum(y for _, y in self.points) / n
        sxx = sum((x - mx) ** 2 for x, _ in self.points)
        syy = sum((y - my) ** 2 for _, y in self.points)
        sxy = sum((x - mx) * (y - my) for x, y in self.points)
        axis = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        c, s = math.cos(axis), math.sin(axis)
        (x0, y0), (x1, y1) = self.points[0], self.points[-1]
        if (x1 - x0) * c + (y1 - y0) * s < 0.0:
            axis += math.pi
            c, s = -c, -s

        along = [(x - mx) * c + (y - my) * s for x, y in self.points]
        across = [-(x - mx) * s + (y - my) * c for x, y in self.points]
        length = max(along) - min(along)
        rms = math.sqrt(sum(r * r for r in across) / n)
        max_residual = max(abs(r) for r in across)
        ratio = length / self.odom_distance

        travel = axis if self.direction > 0.0 else axis + math.pi
        mean_offset = sum(self.yaw_offsets) / len(self.yaw_offsets)
        heading = wrap(travel + self.latest_yaw_offset - mean_offset)
        sigma = max(
            math.sqrt(self.variance_inflation * 12.0 * rms ** 2 / (n * max(length, 1e-3) ** 2)),
            self.min_sigma)
        correction = None if self.global_yaw is None else wrap(heading - self.global_yaw)

        reason = None
        if n < self.min_samples:
            reason = f'N < {self.min_samples}'
        elif length < self.min_length:
            reason = f'shorter than {self.min_length:.2f} m'
        elif rms > self.max_rms:
            reason = f'rms > {self.max_rms:.3f} m'
        elif max_residual > self.max_residual:
            reason = f'UWB jump {max_residual:.2f} m'
        elif not self.min_length_ratio <= ratio <= self.max_length_ratio:
            reason = 'UWB/odom length mismatch'
        elif correction is not None and abs(correction) > self.max_correction:
            reason = f'correction > {math.degrees(self.max_correction):.0f} deg'

        ekf = 'n/a' if correction is None else (
            f'{math.degrees(self.global_yaw):.1f} deg (diff {math.degrees(correction):+.1f})')
        self.get_logger().info(
            f'segment L={length:.2f} m N={n} heading={math.degrees(heading):.1f} deg '
            f'sigma={math.degrees(sigma):.1f} deg rms={rms:.3f} m ratio={ratio:.2f} '
            f'ekf_yaw={ekf} -> {"ACCEPTED" if reason is None else "rejected: " + reason} '
            f'[end: {end_reason}]')
        if reason is not None:
            return

        out = PoseWithCovarianceStamped()
        out.header.stamp = self.latest_stamp
        out.header.frame_id = 'map'
        out.pose.pose.position.x = x1
        out.pose.pose.position.y = y1
        out.pose.pose.orientation.z = math.sin(heading / 2.0)
        out.pose.pose.orientation.w = math.cos(heading / 2.0)
        out.pose.covariance = [0.0] * 36
        for i in (0, 7, 14, 21, 28):
            out.pose.covariance[i] = 1e6
        out.pose.covariance[35] = sigma ** 2
        self.heading_pub.publish(out)

        self.last_accepted = self.now_sec()
        self.request_pending = False

    def request_tick(self):
        if self.active_requests:
            now = self.now_sec()
            if (now - self.last_accepted >= self.request_interval
                    and now - self.last_request >= self.request_retry_interval):
                self.request_id += 1
                self.request_pending = True
                self.last_request = now
                self.get_logger().info(
                    f'No accepted heading for {now - self.last_accepted:.0f} s - '
                    f'requesting a straight calibration drive (#{self.request_id})')
        self.request_pub.publish(UInt32(data=self.request_id if self.request_pending else 0))


def main(args=None):
    rclpy.init(args=args)
    node = UwbHeadingEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
