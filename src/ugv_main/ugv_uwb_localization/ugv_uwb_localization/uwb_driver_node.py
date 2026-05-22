"""
Adapter node: dwm1001_transform Odometry  →  PoseWithCovarianceStamped (/uwb/pose)

The dwm1001_ros2 pipeline publishes nav_msgs/Odometry from the
dwm1001_transform package.  The existing EKF (ekf_global_filter_node)
expects geometry_msgs/PoseWithCovarianceStamped on /uwb/pose.

This node bridges those two interfaces so the EKF config stays unchanged
whether the simulator or the real DWM1001C hardware is used.

ROS2 topic flow:
  active_tag       → /uwb/point_raw   (PointStamped,  dwm1001 frame)
  dwm1001_transform → /uwb/odometry   (Odometry,       map frame)
  uwb_driver_node  → /uwb/pose       (PoseWithCovStamped, map frame)
  uwb_driver_node  → /uwb/status     (String, 'ok')
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class UwbDriverNode(Node):
    def __init__(self):
        super().__init__('uwb_driver_node')

        self.declare_parameter('input_topic', '/uwb/odometry')
        self.declare_parameter('pose_topic', '/uwb/pose')
        self.declare_parameter('status_topic', '/uwb/status')
        self.declare_parameter('frame_id', 'map')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        status_topic = self.get_parameter('status_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.sub = self.create_subscription(Odometry, input_topic, self._on_odometry, 10)

        self.get_logger().info(
            f'UWB driver adapter ready. '
            f'Input: {input_topic} → Output: {pose_topic}'
        )

    def _on_odometry(self, msg: Odometry):
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        out.pose.pose.position.x = msg.pose.pose.position.x
        out.pose.pose.position.y = msg.pose.pose.position.y
        out.pose.pose.position.z = 0.0
        out.pose.pose.orientation.w = 1.0

        # Pass through the covariance filled in by dwm1001_transform
        # (set via the position_cov parameter on that node).
        out.pose.covariance = msg.pose.covariance

        self.pose_pub.publish(out)

        status = String()
        status.data = 'ok'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = UwbDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
