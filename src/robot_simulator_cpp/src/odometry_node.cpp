#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
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

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    reset_srv_ = this->create_service<custom_interfaces::srv::ResetPosition>(
      "/ResetPosition",
      std::bind(&OdometryNode::resetCallback, this, std::placeholders::_1, std::placeholders::_2));

    x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
    last_vx_ = 0.0; last_vy_ = 0.0; last_omega_ = 0.0;

    last_time_ = this->now();
    timer_ = this->create_wall_timer(20ms, std::bind(&OdometryNode::update, this));
    RCLCPP_INFO(this->get_logger(), "odometry_node started");
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
<<<<<<< HEAD
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
=======
    last_vx_ = msg->linear.x;
    last_omega_ = msg->angular.z;
  }

  void resetCallback(const std::shared_ptr<custom_interfaces::srv::ResetPosition::Request> request,
                     std::shared_ptr<custom_interfaces::srv::ResetPosition::Response> response) {
>>>>>>> refs/remotes/origin/main
    x_ = request->pose.position.x;
    y_ = request->pose.position.y;
    // extract yaw from quaternion
    double qx = request->pose.orientation.x;
    double qy = request->pose.orientation.y;
    double qz = request->pose.orientation.z;
    double qw = request->pose.orientation.w;
    yaw_ = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    response->success = true;
    RCLCPP_INFO(this->get_logger(), "ResetPosition called: x=%.2f y=%.2f yaw=%.2f", x_, y_, yaw_);
  }

  void update() {
    rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // integrate differential drive kinematics
    double v = last_vx_;
    double omega = last_omega_;

    double dx = v*std::cos(yaw_)*dt;
    double dy = v*std::sin(yaw_)*dt
    double dyaw = omega * dt;

    // update pose in world frame
    x_ += dx;
    y_ += dy:
    yaw_ += dyaw;

    // publish odometry
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "world";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0,0,yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = last_vx_;
    odom.twist.twist.angular.z = last_omega_;
    odom_pub_->publish(odom);

  // append to path and publish
  geometry_msgs::msg::PoseStamped p;
  p.header.stamp = now;
  p.header.frame_id = "world";
  p.pose = odom.pose.pose;
  path_msg_.header.stamp = now;
  path_msg_.header.frame_id = "world";
  path_msg_.poses.push_back(p);
  path_pub_->publish(path_msg_);

    // publish tf
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now;
    t.header.frame_id = "world";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<custom_interfaces::srv::ResetPosition>::SharedPtr reset_srv_;

  nav_msgs::msg::Path path_msg_;

  double x_, y_, yaw_;
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
