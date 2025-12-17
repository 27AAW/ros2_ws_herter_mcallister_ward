#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "custom_interfaces/srv/reset_position.hpp"  // Generated service header

using namespace std::chrono_literals;

class OdometryNode : public rclcpp::Node {
public:
  OdometryNode() : Node("odometry_node"), x_(0.0), y_(0.0), theta_(0.0) {
    RCLCPP_INFO(this->get_logger(), "Node started");
    // Subscribe to /cmd_vel topic
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&OdometryNode::cmdVelCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /cmd_vel");

    // Create service for ResetPosition
    reset_srv_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "reset_position",
      std::bind(&OdometryNode::resetPositionCallback, this,
                std::placeholders::_1, std::placeholders::_2));

     RCLCPP_INFO(this->get_logger(), "Service /reset_position ready");

    // Initialize tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "TF broadcaster initialized");

    last_time_ = this->now();

    // Timer to update odometry and broadcast tf at 50 Hz
    timer_ = this->create_wall_timer(
      20ms, std::bind(&OdometryNode::updateOdometry, this));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // Store commanded velocity for differential drive (only linear.x and angular.z)
    vx_ = msg->linear.x;
    vy_ = 0.0;  // Differential drive cannot move sideways
    vtheta_ = msg->angular.z;
  }

  void updateOdometry() {
    auto current_time = this->now();
    double dt = (current_time - last_time_).seconds();

    // Differential drive odometry integration
    double delta_x = vx_ * std::cos(theta_) * dt;
    double delta_y = vx_ * std::sin(theta_) * dt;
    double delta_theta = vtheta_ * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    last_time_ = current_time;

    // Prepare transform
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;

    // Convert theta to quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();

    // Broadcast tf transform
    tf_broadcaster_->sendTransform(odom_trans);
  }

  void resetPositionCallback(
    const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> request,
    std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> response) {

    // Reset pose to requested position
    x_ = request->pose.position.x;
    y_ = request->pose.position.y;

    // Convert orientation quaternion to yaw (theta)
    tf2::Quaternion q(
      request->pose.orientation.x,
      request->pose.orientation.y,
      request->pose.orientation.z,
      request->pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, theta_);

    // Broadcast updated transform immediately
    auto current_time = this->now();

    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;

    odom_trans.transform.rotation = request->pose.orientation;

    tf_broadcaster_->sendTransform(odom_trans);

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Reset position service called. Pose reset.");
  }

  // Subscribers and services
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Velocity commands
  double vx_{0.0}, vy_{0.0}, vtheta_{0.0};

  // Robot pose
  double x_, y_, theta_;

  rclcpp::Time last_time_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdometryNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
