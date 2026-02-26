#include <sstream>
#include <fstream>
#include <string>
#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "csvLoader.hpp"
#include "pure_pursuit_utils.hpp"

#define ONLINE false

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

// for variables
private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    double lookAhead;
    double K_p;
    double speed;
    std::string waypoints_path;
    std::vector<wayPoint> way_points;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr poseSubscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drivePublisher_;
    std::string parent_frame_id = "map";
    std::string child_frame_id = "zed_camera_link";
    
    Eigen::Affine3d createTransformationMatrix(float x, float y, float z,
                                           float qx, float qy, float qz, float qw) 
    {
        Eigen::Quaterniond quaternion(qw, qx, qy, qz);
        Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
        Eigen::Vector3d translation(x, y, z);
        Eigen::Affine3d transform = Eigen::Translation3d(translation) * rotation_matrix;

        return transform;
    }

public:
    PurePursuit() : Node("pure_pursuit_node")
    {   
        // parameters
        waypoints_path = this->declare_parameter<std::string>("waypoints_path", "src/zedx_pure_pursuit/path/way_points.csv");
        this->declare_parameter<double>("lookAhead", 0.5);
        this->declare_parameter<double>("K_p", 0.5);
        this->declare_parameter<double>("speed", 1.0);

        // TODO: create ROS subscribers and publishers
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        drivePublisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
                          "drive", 10);

        std::string pose_topic = "/localization";
        std::cout << "pose topic: " << pose_topic << std::endl;
        poseSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                          pose_topic, 10, std::bind(&PurePursuit::pose_callback, this, _1));
        std::cout << "node is initialized" << std::endl;
    }

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg)
    {
        std::cout << "pose callback is triggered" << std::endl;

        lookAhead = this->get_parameter("lookAhead").as_double();
        K_p = this->get_parameter("K_p").as_double();
        speed = this->get_parameter("speed").as_double();

        wayPointLoader loadData(waypoints_path.c_str());
        way_points = loadData.way_points;

        // TODO: find the current waypoint to track using methods mentioned in lecture
        // double currPos[2] = {0.0, 0.0};
        // double nextPos[2] = {0.0, 0.0};
        // currPos[0] = pose_msg->pose.pose.position.x;
        // currPos[1] = pose_msg->pose.pose.position.y;

        // convert to geometry::msg format
        geometry_msgs::msg::PointStamped curr_pt_world;
        curr_pt_world.point.x = pose_msg->pose.pose.position.x;
        curr_pt_world.point.y = pose_msg->pose.pose.position.y;
        curr_pt_world.header.frame_id = parent_frame_id;
        geometry_msgs::msg::PointStamped next_pt_world;
        geometry_msgs::msg::PointStamped curr_pt_local;
        geometry_msgs::msg::PointStamped next_pt_local;

        // look up the transformation
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform(
                child_frame_id, parent_frame_id,
                tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_id.c_str(), child_frame_id.c_str(), ex.what());
          return;
        }

        // compute the next look ahead point
        getLookAheadPt(
            curr_pt_world, 
            next_pt_world, 
            curr_pt_local, 
            next_pt_local, 
            lookAhead, 
            way_points, 
            t);

        std::cout << "next_world: " << next_pt_world.point.x << ", " << next_pt_world.point.y << std::endl;
        std::cout << "next: " << next_pt_local.point.x << " curr: " << curr_pt_local.point.x << std::endl;


        // TODO: calculate curvature/steering angle
        if (next_pt_local.point.x - curr_pt_local.point.x < 0)
        {
            std::cout << "Enter the exit condition" << std::endl;
            return;
        }

        double y = next_pt_local.point.y - curr_pt_local.point.y;
        double curvature = (2 * y) / (lookAhead*lookAhead);
        double steeringAngle = K_p*curvature;
        // std::cout << "steeringAngle" << steeringAngle << std::endl;

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        if (steeringAngle < 0.0)
        {
            drive_msg.drive.steering_angle = std::max(steeringAngle, -0.349);
        }
        else
        {
            drive_msg.drive.steering_angle = std::min(steeringAngle, 0.349);
        }
        drive_msg.drive.speed = speed;

        RCLCPP_INFO(this->get_logger(), 
        "Car world: x=%.3f y=%.3f | Next world: x=%.3f y=%.3f | Next local: x=%.3f y=%.3f | Steering: %.3f",
        pose_msg->pose.pose.position.x,
        pose_msg->pose.pose.position.y,
        next_pt_world.point.x,
        next_pt_world.point.y,
        next_pt_local.point.x,
        next_pt_local.point.y,
        steeringAngle);

        drivePublisher_->publish(drive_msg);
    }

    // ~PurePursuit() {}
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}
