#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Time
import math


class MeshMarkerNode(Node):
    def __init__(self):
        super().__init__('mesh_marker')
        self.marker_pub = self.create_publisher(Marker, '/mesh', 2)

        # Create the marker
        self.marker = Marker()
        self.marker.header.frame_id = "map"
        self.marker.ns = ""
        self.marker.id = 0
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.action = Marker.ADD

        # Mesh URL (must be accessible via HTTP)
        self.marker.mesh_resource = "https://raw.githubusercontent.com/jtappen1/meshes/main/model_1.glb"
        self.marker.mesh_use_embedded_materials = True

        # Scale (optional)
        self.marker.scale.x = 1.0
        self.marker.scale.y = 1.0
        self.marker.scale.z = 1.0

        # Color
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0
        # 90 degrees = pi/2
        angle = -math.pi / 2  # -90 degrees
        qx = 0.0
        qy = 0.0
        qz = math.sin(angle/2)
        qw = math.cos(angle/2)

        # Pose
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = qx
        self.marker.pose.orientation.y = qy
        self.marker.pose.orientation.z = qz
        self.marker.pose.orientation.w = qw
        # self.marker.pose.orientation.w = 1.0

        # Publish periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_marker)

    def publish_marker(self):
        # Update timestamp
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker_pub.publish(self.marker)

def main(args=None):
    rclpy.init(args=args)
    node = MeshMarkerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()