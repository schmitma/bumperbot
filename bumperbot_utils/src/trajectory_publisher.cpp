#include "bumperbot_utils/trajectory_publisher.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>

using std::placeholders::_1;

TrajectoryPublisher::TrajectoryPublisher(const std::string& name) : Node(name)
{
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/bumperbot_controller/odom", 10, std::bind(&TrajectoryPublisher::odomCallback, this, _1));
  traj_pub_ = create_publisher<nav_msgs::msg::Path>("/bumperbot_controller/trajectory", 10);
}

void TrajectoryPublisher::odomCallback(const nav_msgs::msg::Odometry& msg)
{
  path_msg_.header.stamp    = get_clock()->now();
  path_msg_.header.frame_id = msg.header.frame_id;

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose   = msg.pose.pose;
  path_msg_.poses.push_back(pose_stamped);

  traj_pub_->publish(path_msg_);
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisher>("trajectory_publisher");
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}