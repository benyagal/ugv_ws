#!/usr/bin/env python3
"""Fake dead-chicken detections for mission planner tests (plan 4.4).

Each RViz "Publish Point" click (/clicked_point) adds a dead chicken. The
full list is republished as geometry_msgs/PoseArray on
/dead_chicken/detections at `rate` Hz, like the planned overhead interface
(all known dead chickens, repeatedly; the mission planner filters repeats).

Service ~/clear (std_srvs/Trigger) empties the list.

Usage: ros2 run ugv_tools fake_dead_chickens
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Pose, PoseArray
from std_srvs.srv import Trigger


class FakeDeadChickens(Node):
    def __init__(self):
        super().__init__('fake_dead_chickens')
        self.frame_id = self.declare_parameter('frame_id', 'map').value
        topic = self.declare_parameter('topic', '/dead_chicken/detections').value
        rate = self.declare_parameter('rate', 1.0).value

        self.points = []
        self.pub = self.create_publisher(PoseArray, topic, 10)
        self.create_subscription(PointStamped, '/clicked_point', self._on_click, 10)
        self.create_service(Trigger, '~/clear', self._on_clear)
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            f"Click 'Publish Point' in RViz (fixed frame {self.frame_id}) to add a dead chicken -> {topic}")

    def _on_click(self, msg):
        if msg.header.frame_id != self.frame_id:
            self.get_logger().warn(
                f"Ignored click in frame '{msg.header.frame_id}', set the RViz fixed frame to '{self.frame_id}'")
            return
        self.points.append((msg.point.x, msg.point.y))
        self.get_logger().info(
            f"Dead chicken #{len(self.points)} at ({msg.point.x:.2f}, {msg.point.y:.2f})")
        self._publish()

    def _on_clear(self, request, response):
        self.points.clear()
        response.success = True
        response.message = 'cleared'
        self.get_logger().info('List cleared')
        return response

    def _publish(self):
        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in self.points:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDeadChickens()
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
