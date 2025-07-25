#include "posture_game/movement_publisher_node.hpp"
#include <algorithm>

MovementController::MovementController()
: Node("movement_controller")
{
  publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/joint_trajectory_controller/joint_trajectory", 10);

  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "/yaren_visual_state", 10,
    std::bind(&MovementController::visualCallback, this, std::placeholders::_1));

  joint_names_.reserve(12);
  for (int i = 1; i <= 12; ++i)
    joint_names_.push_back("joint_" + std::to_string(i));

  poses_ = {
    {"welcome",     {0.0, -0.10, 0.01, 0.00, 0.01, 0.05, 0.00, 0.17, 1.48, 0.05, 0.68, 0.17}},
    {"explanation", {0.00, 0.12, 0.00, 0.00, 1.43, 0.65, 0.18, 0.17, 1.47, 0.68, 0.18, 0.17}},
    {"end",         {0.00, 0.11, 0.01, 0.00, 1.48, 0.05, -0.68, 0.17, 0.00, 0.05, 0.00, 0.17}},
    {"success",     {0.00, -0.10, 0.00, -0.25, 1.49, 0.49, -0.67, 0.17, 1.48, 0.49, 0.68, 0.17}},
    {"error",       {0.00, 0.00, -0.49, -0.01, 0.00, 0.05, 0.00, 0.17, 0.00, 0.05, 0.00, 0.17}}
  };

  RCLCPP_INFO(this->get_logger(), "🎮 MovementController activo y escuchando...");
}

void MovementController::visualCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string pose_name = msg->data;
  std::transform(pose_name.begin(), pose_name.end(), pose_name.begin(), ::tolower);

  if (poses_.find(pose_name) == poses_.end())
  {
    RCLCPP_WARN(this->get_logger(), "⚠️ Postura desconocida: %s", pose_name.c_str());
    return;
  }

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = poses_[pose_name];
  point.time_from_start = rclcpp::Duration::from_seconds(2.0);

  traj_msg.points.push_back(point);
  publisher_->publish(traj_msg);

  RCLCPP_INFO(this->get_logger(), "✅ Postura enviada: %s", pose_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovementController>());
  rclcpp::shutdown();
  return 0;
}

