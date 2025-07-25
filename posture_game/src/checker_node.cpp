#include "posture_game/checker_node.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace posture_game
{

CheckerNode::CheckerNode() : Node("checker_node"), detector_()
{
    result_pub_ = this->create_publisher<std_msgs::msg::Bool>("/pose_result", 10);
    duration_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pose_duration", 10);
    presence_pub_ = this->create_publisher<std_msgs::msg::Bool>("/player_present", 10);

    pose_data_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/current_pose_data", 10,
        std::bind(&CheckerNode::validate_pose_callback, this, _1));

    landmarks_sub_ = this->create_subscription<posture_game_interfaces::msg::HolisticLandmarks>(
        "/holistic_landmarks", 10,
        std::bind(&CheckerNode::landmarks_callback, this, _1));

    presence_timer_ = this->create_wall_timer(
        500ms, std::bind(&CheckerNode::check_presence, this));

    RCLCPP_INFO(this->get_logger(), "✅ Nodo CheckerNode en ejecución.");
}

void CheckerNode::landmarks_callback(const posture_game_interfaces::msg::HolisticLandmarks::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(landmarks_mutex_);
    pose_landmarks_ = parse_landmarks(msg->pose_landmarks);
    left_hand_landmarks_ = parse_landmarks(msg->left_hand_landmarks);
    right_hand_landmarks_ = parse_landmarks(msg->right_hand_landmarks);
}

void CheckerNode::check_presence()
{
    std::lock_guard<std::mutex> lock(landmarks_mutex_);
    bool present = !pose_landmarks_.empty();
    presence_pub_->publish(std_msgs::msg::Bool().set__data(present));
}

void CheckerNode::validate_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 2)
    {
        RCLCPP_ERROR(this->get_logger(), "❌ Mensaje inválido recibido.");
        return;
    }

    int pose_id = static_cast<int>(msg->data[0]);
    float timeout = msg->data[1];

    RCLCPP_INFO(this->get_logger(), "🧠 Validando postura %d por %.1f s", pose_id, timeout);

    stop_flag_ = true;
    if (validation_thread_.joinable())
        validation_thread_.join();

    stop_flag_ = false;
    validation_thread_ = std::thread(&CheckerNode::run_validation_loop, this, pose_id, timeout);
}

void CheckerNode::run_validation_loop(int pose_id, float timeout)
{
    auto start = std::chrono::steady_clock::now();

    while (!stop_flag_)
    {
        auto now = std::chrono::steady_clock::now();
        float elapsed = std::chrono::duration<float>(now - start).count();

        if (elapsed > timeout)
        {
            RCLCPP_INFO(this->get_logger(), "⏱️ Tiempo agotado sin éxito.");
            publish_result(false, elapsed);
            return;
        }

        std::vector<MockLandmark> lm, rh, lh;
        {
            std::lock_guard<std::mutex> lock(landmarks_mutex_);
            lm = pose_landmarks_;
            rh = right_hand_landmarks_;
            lh = left_hand_landmarks_;
        }

        bool pose_ok = false, hand_ok = false;

        if (!lm.empty())
        {
            if (pose_id == 0) pose_ok = detector_.is_right_arm_up(lm);
            else if (pose_id == 1) pose_ok = detector_.is_left_arm_up(lm);
            else if (pose_id == 2) pose_ok = detector_.are_arms_in_x(lm);
            else if (pose_id == 6) pose_ok = detector_.is_right_arm_horizontal(lm);
            else if (pose_id == 7) pose_ok = detector_.is_left_arm_horizontal(lm);
            else if (pose_id == 8) pose_ok = detector_.are_both_arms_horizontal(lm);
            else if (pose_id == 9) pose_ok = detector_.is_wrist_touching_nose(lm, "right") ||
                                            detector_.is_wrist_touching_nose(lm, "left");
        }

        if (pose_id == 3 && !rh.empty())
            hand_ok = detector_.is_right_fist_exclusive(rh, &lh);
        else if (pose_id == 4 && !lh.empty())
            hand_ok = detector_.is_left_fist_exclusive(lh, &rh);
        else if (pose_id == 5 && !rh.empty() && !lh.empty())
            hand_ok = detector_.are_both_fists_closed(rh, lh);

        if (pose_ok || hand_ok)
        {
            float total = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
            RCLCPP_INFO(this->get_logger(), "✅ Postura detectada correctamente en %.2f s", total);
            publish_result(true, total);
            return;
        }

        std::this_thread::sleep_for(100ms);
    }
}

void CheckerNode::publish_result(bool result, float duration)
{
    result_pub_->publish(std_msgs::msg::Bool().set__data(result));
    duration_pub_->publish(std_msgs::msg::Float32().set__data(duration));
    RCLCPP_INFO(this->get_logger(), "📤 Resultado publicado: %s (%.2fs)",
                result ? "✅ Correcto" : "❌ Incorrecto", duration);
}

}  // namespace posture_game

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<posture_game::CheckerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
