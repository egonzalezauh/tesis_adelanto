#ifndef POSTURE_GAME_CHECKER_NODE_HPP_
#define POSTURE_GAME_CHECKER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "posture_game/postures.hpp"
#include "posture_game_interfaces/msg/holistic_landmarks.hpp"

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <cmath>
#include <mutex>

namespace posture_game
{

class CheckerNode : public rclcpp::Node
{
public:
    CheckerNode();

private:
    // Clase de validación
    PostureDetector detector_;
    std::vector<MockLandmark> pose_landmarks_, left_hand_landmarks_, right_hand_landmarks_;
    std::mutex landmarks_mutex_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr result_pub_, presence_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr duration_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pose_data_sub_;
    rclcpp::Subscription<posture_game_interfaces::msg::HolisticLandmarks>::SharedPtr landmarks_sub_;
    rclcpp::TimerBase::SharedPtr presence_timer_;

    // Validación
    std::thread validation_thread_;
    std::atomic_bool stop_flag_{false};

    // Métodos
    void validate_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void landmarks_callback(const posture_game_interfaces::msg::HolisticLandmarks::SharedPtr msg);
    void run_validation_loop(int pose_id, float timeout);
    void publish_result(bool result, float duration);
    void check_presence();
};

}  // namespace posture_game

#endif  // POSTURE_GAME_CHECKER_NODE_HPP_
