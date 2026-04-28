#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import pyzed.sl as sl
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import threading
import time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import Path
import copy

MIN_DIST_SQ = 1.0 ** 2

RESOLUTIONS = {
    "HD1200":   sl.RESOLUTION.HD1200,
    "HD1080": sl.RESOLUTION.HD1080,
    "SVGA":   sl.RESOLUTION.SVGA,
}

STATUS_MAP = {
    sl.SPATIAL_MEMORY_STATUS.INITIALIZING: "INITIALIZING",
    sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:    "KNOWN_MAP",
    sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:  "LOOP_CLOSED",
    sl.SPATIAL_MEMORY_STATUS.LOST:         "LOST",
    sl.SPATIAL_MEMORY_STATUS.MAP_UPDATE:   "MAP_UPDATE",
}

DEPTH_MODE = {
    "NEURAL_LIGHT": sl.DEPTH_MODE.NEURAL_LIGHT,
    "NEURAL": sl.DEPTH_MODE.NEURAL,
    "NEURAL_PLUS": sl.DEPTH_MODE.NEURAL_PLUS
}

class ZEDSLAMNode(Node):
    def __init__(self):
        super().__init__('zed_positional_tracking_node')

        # -------------- Publishers ---------------------
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)
        self.status_pub = self.create_publisher(DiagnosticArray, '/zed/spatial_memory_status', 10)
        self.path_pub = self.create_publisher(Path, '/zed/path', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Load Config ----------------
        self.declare_parameter('fps', 30)
        self.declare_parameter('resolution', 'SVGA')
        self.declare_parameter('depth_mode', 'NEURAL_LIGHT')
        self.declare_parameter('area_file', '')
        self.declare_parameter('initial_mapping', False)
        self.declare_parameter('update_map', True)

        self.fps = self.get_parameter('fps').value
        self.resolution = self.get_parameter('resolution').value
        self.area_file = self.get_parameter('area_file').value
        self.initial_mapping = self.get_parameter('initial_mapping').value
        self.update_map = self.get_parameter('update_map').value
        self.depth_mode = self.get_parameter('depth_mode').value

        # ---------------- Message Init ----------------
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "map"
        self.tform = TransformStamped()
        self.tform.header.frame_id = "map"
        self.tform.child_frame_id = "zed_camera_link"
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"    
        self.last_mem_status = None
        
        # ---------------- Camera Init ----------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = RESOLUTIONS[self.resolution]
        init_params.camera_fps = self.fps
        init_params.coordinate_units = sl.UNIT.METER
        init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        init_params.depth_mode = DEPTH_MODE[self.depth_mode]

        if self.zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error("ZED Camera failed to open!")
            rclpy.shutdown()
            return

        # ---------------- Tracking Init ----------------
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_3
        tracking_params.enable_area_memory = True
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_2d_ground_mode = False
        tracking_params.enable_localization_only = False

        if self.area_file:
            if not self.initial_mapping:
                tracking_params.area_file_path = self.area_file
                self.get_logger().info(f"Loading Map with Area File: {self.area_file}")
            else:
                self.get_logger().info(f"Initial Mapping with Area File: {self.area_file}")

        self.zed.enable_positional_tracking(tracking_params)
        self.runtime_params = sl.RuntimeParameters()
        self.pose = sl.Pose()

        self.running = True

        self.grab_thread = threading.Thread(target=self.grab_loop)
        self.grab_thread.start()
        self.get_logger().info("ZED Positional Tracking Node started")

    def _moved_enough(self, x, y, z):
        if not self.path_msg.poses:
            return True
        last = self.path_msg.poses[-1].pose.position
        dx, dy, dz = x - last.x, y - last.y, z - last.z
        return dx*dx + dy*dy + dz*dz > MIN_DIST_SQ

    def publish_path(self):
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def grab_loop(self):
        while self.running and rclpy.ok():
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                continue

            state = self.zed.get_position(self.pose)
            if state != sl.POSITIONAL_TRACKING_STATE.OK:
                self.get_logger().warn_once("Tracking lost")
                continue
            
            mem_status = self.zed.get_positional_tracking_status().spatial_memory_status

            if mem_status != self.last_mem_status:
                self.get_logger().info(f"Memory status changed: {STATUS_MAP.get(mem_status, 'UNKNOWN')}")
                self.last_mem_status = mem_status

            #--------- Mapping Status Publish -------------
            msg = DiagnosticArray()
            msg.header.stamp = self.get_clock().now().to_msg()
            status = DiagnosticStatus()
            status.message = STATUS_MAP.get(mem_status, "UNKNOWN")
            msg.status = [status]
            self.status_pub.publish(msg)

            # ---------------- Pose Publish ----------------
            t = self.pose.get_translation(sl.Translation())
            x, y, z = t.get()
            o = self.pose.get_orientation(sl.Orientation())
            x_or, y_or, z_or, w_or = o.get()

            stamp = self.get_clock().now().to_msg()
            self.pose_msg.header.stamp = stamp
            self.pose_msg.pose.position.x = x
            self.pose_msg.pose.position.y = y
            self.pose_msg.pose.position.z = z
            self.pose_msg.pose.orientation.x = x_or
            self.pose_msg.pose.orientation.y = y_or
            self.pose_msg.pose.orientation.z = z_or
            self.pose_msg.pose.orientation.w = w_or

            self.pose_pub.publish(self.pose_msg)

            # ---------------- Transform Publish ----------------
            self.tform.header.stamp = stamp
            self.tform.transform.translation.x = x
            self.tform.transform.translation.y = y
            self.tform.transform.translation.z = z
            self.tform.transform.rotation = self.pose_msg.pose.orientation

            self.tf_broadcaster.sendTransform(self.tform)

            self.path_msg.poses = self.path_msg.poses[-500:]

            if self._moved_enough(x, y, z):
                self.path_msg.poses.append(copy.deepcopy(self.pose_msg))
                self.publish_path()


    # ---------------- Shutdown ----------------
    def destroy_node(self):
        self.running = False
        self.grab_thread.join()
        
        if self.update_map and self.area_file:
            self.get_logger().info("Saving area map before shutdown...")
            self.zed.save_area_map(self.area_file)

        self.grab_thread.join()
        self.zed.disable_positional_tracking()
        self.zed.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZEDSLAMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()