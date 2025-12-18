#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "custom_interfaces/srv/reset_position.hpp"

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node {
public:
  OdometryNode(): Node("odometry_node") {
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&OdometryNode::cmdVelCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/world", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    reset_srv_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition",
      std::bind(&OdometryNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize pose and velocities
    x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
    vx_ = 0.0; vy_ = 0.0; vtheta_ = 0.0;
    last_vx_ = 0.0; last_vy_ = 0.0; last_omega_ = 0.0;

    last_time_ = this->now();
    timer_ = this->create_wall_timer(20ms, std::bind(&OdometryNode::update, this));
    path_msg_.header.frame_id = "world";
    
    RCLCPP_INFO(this->get_logger(), "odometry_node started");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Store commanded velocity for differential drive (only linear.x and angular.z)
    vx_ = msg->linear.x;
    vy_ = 0.0;  // Differential drive cannot move sideways
    vtheta_ = msg->angular.z;
  }

  void resetCallback(const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> request,
                     std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> response) {
    x_ = request->pose.position.x;
    y_ = request->pose.position.y;
    // extract yaw from quaternion
    double qx = request->pose.orientation.x;
    double qy = request->pose.orientation.y;
    double qz = request->pose.orientation.z;
    double qw = request->pose.orientation.w;
    yaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    vx_ = 0.0; vy_ = 0.0; vtheta_ = 0.0;
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "ResetPosition called: x=%.2f y=%.2f yaw=%.2f", x_, y_, yaw_);
  }

  void update() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt > 0.1) return; // Skip if dt too large

    // Differential drive kinematics (project requirement)
    double delta_x = vx_ * std::cos(yaw_) * dt;
    double delta_y = vx_ * std::sin(yaw_) * dt;
    double delta_yaw = vtheta_ * dt;

    // Update pose
    x_ += delta_x;
    y_ += delta_y;
    yaw_ += delta_yaw;

    // Publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    
    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = vtheta_;
    odom_pub_->publish(odom);

    // Append to path
    geometry_msgs::msg::PoseStamped p;
    p.header.stamp = now;
    p.header.frame_id = "world";
    p.pose = odom.pose.pose;
    path_msg_.poses.push_back(p);
    path_msg_.header.stamp = now;
    path_pub_->publish(path_msg_);

    // Publish TF
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }

  // Subscriptions, publishers, timers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_srv_;

  // Path message
  nav_msgs::msg::Path path_msg_;

  // Pose and velocity state
  double x_, y_, yaw_;
  double vx_, vy_, vtheta_;           // Current commanded velocities
  double last_vx_, last_vy_, last_omega_;
  rclcpp::Time last_time_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
