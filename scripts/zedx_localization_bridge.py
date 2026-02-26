#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class ZedxLocalizationBridge(Node):

    def __init__(self):
        super().__init__('zed_localization_bridge')

        # self.broadcaster = StaticTransformBroadcaster(self)

        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'zed_camera_link'
        # t.child_frame_id = 'base_link'

        # t.transform.translation.x = 0.0
        # t.transform.translation.y = 0.08
        # t.transform.translation.z = 0.0

        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = 0.0
        # t.transform.rotation.w = 1.0

        # self.broadcaster.sendTransform(t)
        # self.get_logger().info('Static transform published: zed_camera_link -> base_link')

        self.pose_sub = self.create_subscription(
            PoseStamped,
            "/zed/zed_node/pose",
            self.pose_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization",
            10
        )

    
    def pose_callback(self, pose_msg):
        base_msg = PoseWithCovarianceStamped()
        base_msg.header.stamp = self.get_clock().now().to_msg()
        base_msg.header.frame_id = "map"
        base_msg.pose.pose = pose_msg.pose
        base_msg.pose.covariance = [0.0]*36
        self.pose_pub.publish(base_msg)


def main():
    rclpy.init()
    node = ZedxLocalizationBridge()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
