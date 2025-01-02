#ifndef TRAJECTORY_PUBLISHER_HPP
#define TRAJECTORY_PUBLISHER_HPP

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>

class TrajectoryPublisher : public rclcpp::Node
{
 public:
  TrajectoryPublisher(const std::string& name);

 private:
  void odomCallback(const nav_msgs::msg::Odometry& msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub_;

  nav_msgs::msg::Path path_msg_;
};

#endif