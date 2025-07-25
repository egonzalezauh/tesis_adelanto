#pragma once

#include <vector>
#include <string>

namespace posture_game
{

struct MockLandmark
{
    float x;
    float y;
    float z;
    float visibility;

    MockLandmark(float x_val, float y_val, float z_val, float vis = 1.0f);
};

std::vector<MockLandmark> parse_landmarks(const std::vector<float> &flat_array);

class PostureDetector
{
public:
    PostureDetector();

    bool is_fist_closed(const std::vector<MockLandmark> &lm) const;
    bool is_right_fist_exclusive(const std::vector<MockLandmark> &right_hand,
                                 const std::vector<MockLandmark> *left_hand = nullptr) const;
    bool is_left_fist_exclusive(const std::vector<MockLandmark> &left_hand,
                                const std::vector<MockLandmark> *right_hand = nullptr) const;
    bool are_both_fists_closed(const std::vector<MockLandmark> &right_hand,
                               const std::vector<MockLandmark> &left_hand) const;
    bool is_right_arm_horizontal(const std::vector<MockLandmark> &lm) const;
    bool is_left_arm_horizontal(const std::vector<MockLandmark> &lm) const;
    bool are_both_arms_horizontal(const std::vector<MockLandmark> &lm) const;
    bool is_right_arm_up(const std::vector<MockLandmark> &lm) const;
    bool is_left_arm_up(const std::vector<MockLandmark> &lm) const;
    bool are_arms_in_x(const std::vector<MockLandmark> &lm) const;
    bool is_wrist_touching_nose(const std::vector<MockLandmark> &lm, const std::string &side = "right", float threshold = -1.0f) const;

private:
    float euclidean(const MockLandmark &p1, const MockLandmark &p2) const;

    // Índices
    const int NOSE = 0;
    const int LEFT_SHOULDER = 11;
    const int RIGHT_SHOULDER = 12;
    const int LEFT_ELBOW = 13;
    const int RIGHT_ELBOW = 14;
    const int LEFT_WRIST = 15;
    const int RIGHT_WRIST = 16;

    // Umbrales
    const float ANGLE_THRESHOLD = 160.0f;
    const float VERTICAL_THRESHOLD = 0.15f;
    const float HORIZONTAL_THRESHOLD = 0.08f;
    const float VERTICAL_DIFF_THRESHOLD = 0.07f;
    const float WRIST_DIST_THRESHOLD = 0.1f;
    const float SIMILAR_HEIGHT_THRESHOLD = 0.08f;
    const float NOSE_TOUCH_THRESHOLD = 0.10f;
};

} // namespace posture_game
