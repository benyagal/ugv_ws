#!/usr/bin/env python3
"""Bridges RViz's "2D Pose Estimate" tool to robot_localization's EKF nodes.

RViz's "2D Pose Estimate" tool publishes a PoseWithCovarianceStamped on
/initialpose, but robot_localization's ekf_node does NOT subscribe to that
topic directly - it only exposes a 'set_pose' service
(robot_localization/srv/SetPose). This node subscribes to /initialpose and
calls that service on BOTH the local and global EKF filters, so that
setting an initial pose in RViz works directly without a manual
`ros2 service call`.

Both filters need to be reset together: if only the global (map-frame)
filter were reset, the local filter would keep publishing its old,
unreset yaw on /odometry/local, which the global filter fuses as an
absolute measurement - immediately pulling the just-set orientation back
towards the old, wrong value on the very next update.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from robot_localization.srv import SetPose


class InitialPoseBridge(Node):
    def __init__(self):
        super().__init__('initialpose_bridge')

        self.local_client = self.create_client(SetPose, '/ekf_local/set_pose')
        self.global_client = self.create_client(SetPose, '/ekf_global/set_pose')

        self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10
        )

        self.get_logger().info('Bridging RViz "2D Pose Estimate" (/initialpose) to EKF set_pose services.')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        # Global filter (map frame): use the pose exactly as given.
        global_req = SetPose.Request()
        global_req.pose = msg
        global_req.pose.header.frame_id = 'map'
        self._call_when_ready(self.global_client, global_req, 'global')

        # Local filter (odom frame): only the orientation matters here (the
        # odom frame's origin is arbitrary), so reset position to (0, 0)
        # and keep the same orientation, to stay in sync with the global
        # filter's newly-set yaw.
        local_req = SetPose.Request()
        local_req.pose = msg
        local_req.pose.header.frame_id = 'odom'
        local_req.pose.pose.pose.position.x = 0.0
        local_req.pose.pose.pose.position.y = 0.0
        local_req.pose.pose.pose.position.z = 0.0
        self._call_when_ready(self.local_client, local_req, 'local')

    def _call_when_ready(self, client, request, label):
        if not client.service_is_ready():
            self.get_logger().warn(f'{label} set_pose service not available yet, skipping this update.')
            return
        client.call_async(request)
        self.get_logger().info(f'Sent set_pose to {label} EKF.')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
