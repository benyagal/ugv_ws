#!/usr/bin/env python3
"""Fake overhead-camera chickens for testing chicken_layer without the camera Jetson.

Publishes geometry_msgs/PoseArray (frame map) on /overhead/chickens. The chickens
spawn around the robot's position at startup (or in `area` = [x_min, y_min,
x_max, y_max] if given) and random-walk inside that box.
"""
import math
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class FakeChickens(Node):
    def __init__(self):
        super().__init__('fake_chickens')
        self.count = self.declare_parameter('count', 6).value
        self.area = list(self.declare_parameter('area', [0.0]).value)
        self.spawn_radius = self.declare_parameter('spawn_radius', 2.0).value
        self.keep_clear = self.declare_parameter('keep_clear', 0.6).value
        self.speed = self.declare_parameter('speed', 0.1).value
        self.rate = self.declare_parameter('rate', 5.0).value
        self.frame = self.declare_parameter('frame_id', 'map').value
        self.topic = self.declare_parameter('topic', '/overhead/chickens').value

        self.pub = self.create_publisher(PoseArray, self.topic, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.chickens = []
        self.create_timer(1.0 / self.rate, self.tick)

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_footprint', Time())
        except TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def spawn(self):
        if len(self.area) != 4:
            robot = self.robot_xy()
            if robot is None:
                self.get_logger().info(
                    f'waiting for {self.frame} -> base_footprint TF', throttle_duration_sec=5.0)
                return
            x, y = robot
            r = self.spawn_radius
            self.area = [x - r, y - r, x + r, y + r]
            centre = robot
        else:
            centre = ((self.area[0] + self.area[2]) / 2, (self.area[1] + self.area[3]) / 2)

        while len(self.chickens) < self.count:
            x = random.uniform(self.area[0], self.area[2])
            y = random.uniform(self.area[1], self.area[3])
            if math.hypot(x - centre[0], y - centre[1]) >= self.keep_clear:
                self.chickens.append([x, y, random.uniform(-math.pi, math.pi)])
        self.get_logger().info(
            f'{self.count} fake chickens in x {self.area[0]:.1f}..{self.area[2]:.1f}, '
            f'y {self.area[1]:.1f}..{self.area[3]:.1f} on {self.topic}')

    def tick(self):
        if not self.chickens:
            self.spawn()
            if not self.chickens:
                return

        step = self.speed / self.rate
        msg = PoseArray()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        for c in self.chickens:
            c[2] += random.gauss(0.0, 0.3)
            c[0] = min(max(c[0] + step * math.cos(c[2]), self.area[0]), self.area[2])
            c[1] = min(max(c[1] + step * math.sin(c[2]), self.area[1]), self.area[3])
            pose = Pose()
            pose.position.x = c[0]
            pose.position.y = c[1]
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeChickens()
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
