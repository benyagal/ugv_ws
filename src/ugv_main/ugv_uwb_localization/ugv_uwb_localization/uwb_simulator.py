import math
import random

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class UwbSimulator(Node):
    def __init__(self):
        super().__init__('uwb_simulator')

        self.declare_parameter('source_topic', '/odometry/local')
        self.declare_parameter('pose_topic', '/uwb/pose')
        self.declare_parameter('status_topic', '/uwb/status')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('position_noise_stddev', 0.05)
        self.declare_parameter('dropout_probability', 0.05)
        self.declare_parameter('outlier_probability', 0.03)
        self.declare_parameter('outlier_distance', 1.0)
        self.declare_parameter('position_offset_x', 0.0)
        self.declare_parameter('position_offset_y', 0.0)

        source_topic = self.get_parameter('source_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.position_noise_stddev = self.get_parameter('position_noise_stddev').get_parameter_value().double_value
        self.dropout_probability = self.get_parameter('dropout_probability').get_parameter_value().double_value
        self.outlier_probability = self.get_parameter('outlier_probability').get_parameter_value().double_value
        self.outlier_distance = self.get_parameter('outlier_distance').get_parameter_value().double_value
        self.position_offset_x = self.get_parameter('position_offset_x').get_parameter_value().double_value
        self.position_offset_y = self.get_parameter('position_offset_y').get_parameter_value().double_value

        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.source_subscription = self.create_subscription(Odometry, source_topic, self.handle_source_odometry, 10)

        self.latest_x = None
        self.latest_y = None

        timer_period = 1.0 / max(publish_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def handle_source_odometry(self, msg: Odometry):
        self.latest_x = msg.pose.pose.position.x
        self.latest_y = msg.pose.pose.position.y

    def publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_publisher.publish(msg)

    def publish_pose(self):
        if self.latest_x is None or self.latest_y is None:
            self.publish_status('no_source')
            return

        if random.random() < self.dropout_probability:
            self.publish_status('dropout')
            return

        noisy_x = self.latest_x + self.position_offset_x + random.gauss(0.0, self.position_noise_stddev)
        noisy_y = self.latest_y + self.position_offset_y + random.gauss(0.0, self.position_noise_stddev)
        variance = max(self.position_noise_stddev ** 2, 1e-4)
        status = 'ok'

        if random.random() < self.outlier_probability:
            angle = random.uniform(-math.pi, math.pi)
            noisy_x += math.cos(angle) * self.outlier_distance
            noisy_y += math.sin(angle) * self.outlier_distance
            # NE inflálódjon a variancia: így a Mahalanobis-távolság
            # (ugrás / stddev) magas lesz, és az EKF rejection_threshold
            # alapján kiszűri az outliert. Ha a varianciát is növelnénk,
            # az EKF elfogadná a nagy ugrást "bizonytalan mérésként".
            status = 'outlier'

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.pose.position.x = noisy_x
        msg.pose.pose.position.y = noisy_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        covariance = [0.0] * 36
        covariance[0] = variance
        covariance[7] = variance
        covariance[14] = 1e6
        covariance[21] = 1e6
        covariance[28] = 1e6
        covariance[35] = 1e6
        msg.pose.covariance = covariance

        self.pose_publisher.publish(msg)
        self.publish_status(status)


def main(args=None):
    rclpy.init(args=args)
    node = UwbSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()