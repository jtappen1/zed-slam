#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time
from collections import deque

class PoseRateNode(Node):
    def __init__(self):
        super().__init__('pose_rate_node')
        # Replace with your actual pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/zed/zed_node/pose',
            self.pose_callback,
            10
        )
        # Keep timestamps of recent messages
        self.timestamps = deque()
        self.window_seconds = 1.0  # 1-second moving window

    def pose_callback(self, msg):
        now = time.time()
        self.timestamps.append(now)

        # Remove old timestamps outside the window
        while self.timestamps and self.timestamps[0] < now - self.window_seconds:
            self.timestamps.popleft()

        # Compute rate as messages per second in the window
        hz = len(self.timestamps) / self.window_seconds
        self.get_logger().info(f"Pose rate (1s avg): {hz:.2f} Hz")

def main(args=None):
    rclpy.init(args=args)
    node = PoseRateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()