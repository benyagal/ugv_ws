#!/usr/bin/env python3
"""Simple cmd_vel -> odom integrator for visualization.

Publishes /odom (nav_msgs/Odometry) and broadcasts tf from 'odom' -> 'base_footprint'.
Use for RViz-only simulation when no real base driver is available.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
import time
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmdvel_to_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_rate', 50.0)

        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0

        self.last_time = self.get_clock().now()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1.0 / self.rate, self.timer_cb)

        self.get_logger().info('cmdvel_to_odom started (publishing %s -> %s)' % (self.odom_frame, self.base_frame))

    def cmd_cb(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vth = msg.angular.z

    def timer_cb(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return

        # integrate
        delta_x = (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt
        delta_y = (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt
        delta_yaw = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # publish TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, float(self.yaw))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.broadcaster.sendTransform(t)

        # publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = float(self.vx)
        odom.twist.twist.linear.y = float(self.vy)
        odom.twist.twist.angular.z = float(self.vth)

        self.odom_pub.publish(odom)

        self.last_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
