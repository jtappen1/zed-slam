#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import pyzed.sl as sl
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import argparse


class ZEDPositionalTrackingNode(Node):
    def __init__(self):
        super().__init__('zed_positional_tracking_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)
    
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Camera Init ----------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = 15
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = sl.DEPTH_MODE.NEURAL_LIGHT

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Camera failed to open!")
            rclpy.shutdown()
            return

        # self.update_map = args.update_map
        # self.area_file = args.area_file
        # ---------------- Tracking Init ----------------
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking_params.enable_area_memory = True
        tracking_params.enable_localization_only = False
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_2d_ground_mode = False
        # print(f"AREA FILE: {self.area_file}")
        # print(f"Save Map: {self.update_map}")
        tracking_params.area_file_path = "/home/nvidia/zed_ws/src/zedx_pure_pursuit/path/levine_hallway.area"

        self.zed.enable_positional_tracking(tracking_params)

        self.runtime_params = sl.RuntimeParameters()
        self.pose = sl.Pose()

        # Wait for tracking
        self.get_logger().info("Waiting for localization...")
        while True:
            if self.zed.grab(self.runtime_params) == sl.ERROR_CODE.SUCCESS:
                state = self.zed.get_position(self.pose)
                if state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.get_logger().info("Localization ready.")
                    break

        # ---------------- Start main loop ----------------
        self.main_loop()

    # ---------------- Main Loop ----------------
    def main_loop(self):
        try:
            while rclpy.ok():
                if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                    continue
                
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

                state = self.zed.get_position(self.pose)
                if state != sl.POSITIONAL_TRACKING_STATE.OK:
                    self.get_logger().warn("Tracking lost")
                    continue

                t = self.pose.get_translation(sl.Translation())
                x, y, z = t.get()
                o = self.pose.get_orientation(sl.Orientation())
                x_or, y_or, z_or, w_or = o.get()

                # Build PoseStamped
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"

                # Coordinate alignment
                msg.pose.position.x = y
                msg.pose.position.y = -x
                msg.pose.position.z = z

                # Identity orientation
                msg.pose.orientation.x = x_or
                msg.pose.orientation.y = y_or
                msg.pose.orientation.z = z_or
                msg.pose.orientation.w = w_or

                self.pose_pub.publish(msg)

                tform = TransformStamped()
                tform.header.stamp = msg.header.stamp
                tform.header.frame_id = "map"
                tform.child_frame_id = "zed_camera_link"

                tform.transform.translation.x = msg.pose.position.x
                tform.transform.translation.y = msg.pose.position.y
                tform.transform.translation.z = msg.pose.position.z

                tform.transform.rotation = msg.pose.orientation

                self.tf_broadcaster.sendTransform(tform)
                rclpy.spin_once(self, timeout_sec=0)

        except KeyboardInterrupt:
            pass
            # if self.update_map:
            #     self.get_logger().info("Stopping tracking. Saving map...")
            #     self.zed.save_area_map(self.area_file)
            
        finally:
            self.destroy_node()

    def destroy_node(self):
        self.get_logger().info("Shutting down ZED tracking...")
        self.zed.disable_positional_tracking()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    # parser = argparse.ArgumentParser()
    # parser.add_argument(
    #     "--update-map",
    #     action="store_true",
    #     help="Update area file"
    # )
    # parser.add_argument(
    #     "--area-file",
    #     type=str,
    #     default="",
    #     help="Path to ZED area file"
    # )
    # parsed_args, ros_args = parser.parse_known_args()
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