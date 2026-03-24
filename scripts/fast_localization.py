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
from std_msgs.msg import String



class ZEDPositionalTrackingNode(Node):
    def __init__(self):
        super().__init__('zed_positional_tracking_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/zed/zed_node/pose', 10)
        self.status_pub = self.create_publisher(String, '/zed/spatial_memory_status', 10)
        self.last_mem_status = None

        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------------- Load Config ----------------
        pkg_path = get_package_share_directory('zedx_pure_pursuit')
        yaml_path = os.path.join(pkg_path, 'config', 'localization.yaml')
        self.declare_parameter('yaml_path', yaml_path)
        yaml_path = self.get_parameter('yaml_path').value
        with open(yaml_path, 'r') as f:
            config = yaml.safe_load(f)

        self.area_file = config.get("area_file")
        self.update_map = config.get("update_map", False)
        self.fps = config.get("fps", 30)

        # ---------------- Camera Init ----------------
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.SVGA
        init_params.camera_fps = self.fps
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
        tracking_params.enable_area_memory = True
        tracking_params.enable_localization_only = False
        tracking_params.enable_imu_fusion = True
        tracking_params.set_gravity_as_origin = True
        tracking_params.enable_2d_ground_mode = False
        tracking_params.enable_localization_only = False

        if self.area_file:
            if not config.get("mapping", False):
                tracking_params.area_file_path = self.area_file
                self.get_logger().info(f"Loading Map with Area File: {self.area_file}")
            else:
                self.get_logger().info(f"Initial Mapping with Area File: {self.area_file}")

        self.zed.enable_positional_tracking(tracking_params)
        self.runtime_params = sl.RuntimeParameters()
        self.pose = sl.Pose()

        self.running = True



        # ---------------- Start Grab Thread ----------------
        self.grab_thread = threading.Thread(target=self.grab_loop)
        self.grab_thread.start()
        self.get_logger().info("ZED Positional Tracking Node started")

    # ---------------- Grab Loop ----------------
    def grab_loop(self):
        while self.running and rclpy.ok():
            if self.zed.grab(self.runtime_params) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.001)
                continue

            state = self.zed.get_position(self.pose)
            if state != sl.POSITIONAL_TRACKING_STATE.OK:
                self.get_logger().warn_once("Tracking lost")
                time.sleep(0.001)
                continue
            
            mem_status = self.zed.get_positional_tracking_status().spatial_memory_status
            status_msg = String()

            if mem_status != self.last_mem_status:
                if mem_status == sl.SPATIAL_MEMORY_STATUS.INITIALIZING:
                    status_msg.data = "INITIALIZING"
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.KNOWN_MAP:
                    status_msg.data = "KNOWN_MAP"
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOOP_CLOSED:
                    status_msg.data = "LOOP_CLOSED"
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.LOST:
                    status_msg.data = "LOST"
                elif mem_status == sl.SPATIAL_MEMORY_STATUS.MAP_UPDATE:
                    status_msg.data = "MAP_UPDATE"
                else:
                    status_msg.data = "UNKNOWN"

                self.status_pub.publish(status_msg)
                self.last_mem_status = mem_status

            # ---------------- Pose Publish ----------------
            t = self.pose.get_translation(sl.Translation())
            x, y, z = t.get()
            o = self.pose.get_orientation(sl.Orientation())
            x_or, y_or, z_or, w_or = o.get()

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = y
            msg.pose.position.y = -x
            msg.pose.position.z = z
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



            time.sleep(0.001)  # small sleep to yield CPU

    # ---------------- Shutdown ----------------
    def destroy_node(self):
        self.running = False
        if self.update_map and self.area_file:
            self.get_logger().info("Saving area map before shutdown...")
            self.zed.save_area_map(self.area_file)
        self.grab_thread.join()
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