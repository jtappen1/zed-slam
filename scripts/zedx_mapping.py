#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyzed.sl as sl
import time

class ZEDMapRecordingNode(Node):
    def __init__(self):
        super().__init__('zed_map_recording_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)

        # ---------------- Camera Init ----------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 30
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Camera failed to open!")
            rclpy.shutdown()
            return

        # ---------------- Tracking Init ----------------
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_area_memory = True      # enable mapping
        tracking_params.enable_localization_only = False  # allow recording
        tracking_params.enable_2d_ground_mode = False
        # tracking_params.area_file_path = "/home/nvidia/zed_ws/src/zedx_pure_pursuit/path/upper_levine_test.area"

        self.zed.enable_positional_tracking(tracking_params)
        self.runtime_params = sl.RuntimeParameters()
        self.pose = sl.Pose()

        # Wait until tracking is ready
        self.get_logger().info("Initializing positional tracking...")
        while True:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                state = self.zed.get_position(self.pose)
                if state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.get_logger().info("Positional tracking ready.")
                    break
            time.sleep(0.05)
            print("Initializing")

        # ---------------- Main Loop ----------------
        self.main_loop()

    def main_loop(self):
        try:
            while rclpy.ok():
                if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                    continue

                state = self.zed.get_position(self.pose)
                mem_status = self.zed.get_positional_tracking_status().spatial_memory_status

                # ---------------- Log Spatial Memory Status ----------------
                if mem_status == sl.SPATIAL_MEMORY_STATUS.INITIALIZING:
                    self.get_logger().info("[SPATIAL MEMORY] Initializing...")
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:
                    self.get_logger().info("[SPATIAL MEMORY] Known map")
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:
                    self.get_logger().info("[SPATIAL MEMORY] Loop closure detected!")
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOST:
                    self.get_logger().warn("[SPATIAL MEMORY] Tracking lost")

                if state != sl.POSITIONAL_TRACKING_STATE.OK:
                    self.get_logger().warn("[WARN] Tracking state not OK")
                    continue

                # ---------------- Publish Pose ----------------
                t = self.pose.get_translation(sl.Translation())
                x, y, z = t.get()

                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.pose.position.x = y
                msg.pose.position.y = -x
                msg.pose.position.z = z
                msg.pose.orientation.w = 1.0
                msg.pose.orientation.x = 0.0
                msg.pose.orientation.y = 0.0
                msg.pose.orientation.z = 0.0

                self.pose_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0)

        except KeyboardInterrupt:
            # ---------------- Save Map on Exit ----------------
            self.get_logger().info("Stopping tracking. Saving map...")
            self.zed.save_area_map("/home/nvidia/zed_ws/src/zedx_pure_pursuit/path/levine_hallway.area")
        finally:
            self.destroy_node()

    def destroy_node(self):
        self.get_logger().info("Shutting down ZED tracking...")
        self.zed.disable_positional_tracking()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    ZEDMapRecordingNode()
    rclpy.shutdown()


if __name__ == "__main__":
    main()