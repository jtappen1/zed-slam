#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyzed.sl as sl
import math
import time

def get_yaw(pose):
    rv = pose.get_rotation_vector()
    return math.degrees(rv[1])  # Y component = yaw in RIGHT_HANDED_Y_UP

class ZEDPositionalTrackingNode(Node):
    def __init__(self):
        super().__init__('zed_positional_tracking_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)

        # Initialize ZED camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.HD1080
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Camera failed to open!")
            rclpy.shutdown()
            return

        # Positional tracking parameters (same as your teach script)
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking_params.enable_area_memory = True
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_2d_ground_mode = True
        tracking_params.area_file_path = '/home/nvidia/zed_ws/src/zedx_pure_pursuit/path/my_area.area'

        self.zed.enable_positional_tracking(tracking_params)
        self.tracking_status = self.zed.get_positional_tracking_status()

        self.runtime_params = sl.RuntimeParameters()
        self.pose = sl.Pose()

        # Wait until tracking is ready
        self.get_logger().info("[INFO] Waiting for positional tracking to initialize...")
        while True:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                state = self.zed.get_position(self.pose)
                if state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.get_logger().info("[INFO] Positional tracking ready.")
                    break
            time.sleep(0.1)

        # Start timer to publish pose at ~30 Hz
        self.timer = self.create_timer(1/30.0, self.publish_pose)

    def publish_pose(self):
        if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
            return

        state = self.zed.get_position(self.pose)
        mem_status = self.tracking_status.spatial_memory_status

        # Log spatial memory status
        if mem_status == sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:
            self.get_logger().info("[SPATIAL MEMORY] Loop closure detected!")
        elif mem_status == sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:
            self.get_logger().info("[SPATIAL MEMORY] Known map")
        elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOST:
            self.get_logger().warn("[SPATIAL MEMORY] Tracking lost")
        else:
            self.get_logger().debug(f"[SPATIAL MEMORY] Status {mem_status}")

        if state != sl.POSITIONAL_TRACKING_STATE.OK:
            self.get_logger().warn("[WARN] Tracking state not OK")
            return

        t = self.pose.get_translation(sl.Translation())
        x = t.get()[0]
        y = t.get()[1]
        z = t.get()[2]

        yaw = math.radians(get_yaw(self.pose))  # optional: use in msg orientation

        # Build PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = y      # rotate 90 deg to match your teach script
        msg.pose.position.y = -x
        msg.pose.position.z = z

        # Keep orientation as identity (or compute from rotation matrix if needed)
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0

        self.pose_pub.publish(msg)

    def destroy_node(self):
        self.get_logger().info("[INFO] Shutting down ZED positional tracking...")
        self.zed.disable_positional_tracking()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDPositionalTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()