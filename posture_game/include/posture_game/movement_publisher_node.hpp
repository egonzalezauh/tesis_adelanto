// movement_controller.hpp
#ifndef MOVEMENT_CONTROLLER_HPP_
#define MOVEMENT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

class MovementController : public rclcpp::Node
{
public:
  MovementController();

private:
  void visualCallback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::unordered_map<std::string, std::vector<double>> poses_;
  std::vector<std::string> joint_names_;
};

#endif  // MOVEMENT_CONTROLLER_HPP_
