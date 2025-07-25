#include "posture_game/postures.hpp"
#include <cmath>
#include <algorithm>

namespace posture_game {

MockLandmark::MockLandmark(float x_val, float y_val, float z_val, float vis)
    : x(x_val), y(y_val), z(z_val), visibility(vis) {}

std::vector<MockLandmark> parse_landmarks(const std::vector<float>& flat_array) {
    std::vector<MockLandmark> landmarks;
    for (size_t i = 0; i + 2 < flat_array.size(); i += 3) {
        landmarks.emplace_back(flat_array[i], flat_array[i + 1], flat_array[i + 2]);
    }
    return landmarks;
}

PostureDetector::PostureDetector() {}

float PostureDetector::euclidean(const MockLandmark& p1, const MockLandmark& p2) const {
    float dx = p1.x - p2.x;
    float dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool PostureDetector::is_fist_closed(const std::vector<MockLandmark>& lm) const {
    std::vector<int> tips = {8, 12, 16, 20};
    std::vector<int> pips = {6, 10, 14, 18};
    int base = 0;
    int bent_fingers = 0;

    const MockLandmark& base_lm = lm[base];
    for (size_t i = 0; i < tips.size(); ++i) {
        const MockLandmark& tip = lm[tips[i]];
        const MockLandmark& pip = lm[pips[i]];
        if (euclidean(tip, base_lm) < euclidean(pip, base_lm)) {
            ++bent_fingers;
        }
    }

    const MockLandmark& thumb_tip = lm[4];
    const MockLandmark& thumb_ip = lm[3];
    float thumb_dist = euclidean(thumb_tip, thumb_ip);

    return bent_fingers >= 3 && thumb_dist < 0.05f;
}

bool PostureDetector::is_right_fist_exclusive(const std::vector<MockLandmark>& right_hand,
                                              const std::vector<MockLandmark>* left_hand) const {
    bool right_closed = is_fist_closed(right_hand);
    bool left_closed = left_hand ? is_fist_closed(*left_hand) : false;
    return right_closed && !left_closed;
}

bool PostureDetector::is_left_fist_exclusive(const std::vector<MockLandmark>& left_hand,
                                             const std::vector<MockLandmark>* right_hand) const {
    bool left_closed = is_fist_closed(left_hand);
    bool right_closed = right_hand ? is_fist_closed(*right_hand) : false;
    return left_closed && !right_closed;
}

bool PostureDetector::are_both_fists_closed(const std::vector<MockLandmark>& right_hand,
                                            const std::vector<MockLandmark>& left_hand) const {
    return is_fist_closed(right_hand) && is_fist_closed(left_hand);
}

bool PostureDetector::is_right_arm_horizontal(const std::vector<MockLandmark>& lm) const {
    const MockLandmark& r_shoulder = lm[RIGHT_SHOULDER];
    const MockLandmark& r_elbow = lm[RIGHT_ELBOW];
    const MockLandmark& r_wrist = lm[RIGHT_WRIST];

    const MockLandmark& l_shoulder = lm[LEFT_SHOULDER];
    const MockLandmark& l_elbow = lm[LEFT_ELBOW];
    const MockLandmark& l_wrist = lm[LEFT_WRIST];

    std::vector<float> v1 = {r_shoulder.x - r_elbow.x, r_shoulder.y - r_elbow.y};
    std::vector<float> v2 = {r_wrist.x - r_elbow.x, r_wrist.y - r_elbow.y};

    float dot = v1[0]*v2[0] + v1[1]*v2[1];
    float norm_v1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    float norm_v2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);

    float cos_angle = dot / (norm_v1 * norm_v2);
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
    float angle = std::acos(cos_angle) * (180.0f / M_PI);

    bool altura = std::abs(r_shoulder.y - r_wrist.y) < 0.05f;
    bool brazo_derecho_ok = angle > ANGLE_THRESHOLD && altura;

    bool brazo_izquierdo_abajo = std::abs(l_shoulder.y - l_wrist.y) > 0.12f ||
                                 std::abs(l_elbow.y - l_wrist.y) < 0.05f;

    return brazo_derecho_ok && brazo_izquierdo_abajo;
}

bool PostureDetector::is_left_arm_horizontal(const std::vector<MockLandmark>& lm) const {
    const MockLandmark& l_shoulder = lm[LEFT_SHOULDER];
    const MockLandmark& l_elbow = lm[LEFT_ELBOW];
    const MockLandmark& l_wrist = lm[LEFT_WRIST];

    const MockLandmark& r_shoulder = lm[RIGHT_SHOULDER];
    const MockLandmark& r_elbow = lm[RIGHT_ELBOW];
    const MockLandmark& r_wrist = lm[RIGHT_WRIST];

    std::vector<float> v1 = {l_shoulder.x - l_elbow.x, l_shoulder.y - l_elbow.y};
    std::vector<float> v2 = {l_wrist.x - l_elbow.x, l_wrist.y - l_elbow.y};

    float dot = v1[0]*v2[0] + v1[1]*v2[1];
    float norm_v1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
    float norm_v2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);

    float cos_angle = dot / (norm_v1 * norm_v2);
    cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));
    float angle = std::acos(cos_angle) * (180.0f / M_PI);

    bool altura = std::abs(l_shoulder.y - l_wrist.y) < 0.05f;
    bool brazo_izquierdo_ok = angle > ANGLE_THRESHOLD && altura;

    bool brazo_derecho_abajo = std::abs(r_shoulder.y - r_wrist.y) > 0.12f ||
                               std::abs(r_elbow.y - r_wrist.y) < 0.05f;

    return brazo_izquierdo_ok && brazo_derecho_abajo;
}

bool PostureDetector::are_both_arms_horizontal(const std::vector<MockLandmark>& lm) const {
    return is_right_arm_horizontal(lm) && is_left_arm_horizontal(lm);
}

bool PostureDetector::is_right_arm_up(const std::vector<MockLandmark>& lm) const {
    const MockLandmark& r_shoulder = lm[RIGHT_SHOULDER];
    const MockLandmark& r_elbow = lm[RIGHT_ELBOW];
    const MockLandmark& r_wrist = lm[RIGHT_WRIST];

    const MockLandmark& l_shoulder = lm[LEFT_SHOULDER];
    const MockLandmark& l_elbow = lm[LEFT_ELBOW];
    const MockLandmark& l_wrist = lm[LEFT_WRIST];

    bool right_vertical = (r_shoulder.y - r_wrist.y) > VERTICAL_THRESHOLD;
    bool right_horizontal = std::abs(r_wrist.x - r_shoulder.x) < HORIZONTAL_THRESHOLD;
    bool right_straight = r_wrist.y < r_elbow.y && r_elbow.y < r_shoulder.y;

    bool left_arm_down = l_wrist.y > l_shoulder.y || l_elbow.y > l_shoulder.y;

    return right_vertical && right_horizontal && right_straight && left_arm_down;
}

bool PostureDetector::is_left_arm_up(const std::vector<MockLandmark>& lm) const {
    const MockLandmark& l_shoulder = lm[LEFT_SHOULDER];
    const MockLandmark& l_elbow = lm[LEFT_ELBOW];
    const MockLandmark& l_wrist = lm[LEFT_WRIST];

    const MockLandmark& r_shoulder = lm[RIGHT_SHOULDER];
    const MockLandmark& r_elbow = lm[RIGHT_ELBOW];
    const MockLandmark& r_wrist = lm[RIGHT_WRIST];

    bool left_vertical = (l_shoulder.y - l_wrist.y) > VERTICAL_THRESHOLD;
    bool left_horizontal = std::abs(l_wrist.x - l_shoulder.x) < HORIZONTAL_THRESHOLD;
    bool left_straight = l_wrist.y < l_elbow.y && l_elbow.y < l_shoulder.y;

    bool right_arm_down = r_wrist.y > r_shoulder.y || r_elbow.y > r_shoulder.y;

    return left_vertical && left_horizontal && left_straight && right_arm_down;
}

bool PostureDetector::are_arms_in_x(const std::vector<MockLandmark>& lm) const {
    const MockLandmark& rw = lm[RIGHT_WRIST];
    const MockLandmark& lw = lm[LEFT_WRIST];
    const MockLandmark& rs = lm[RIGHT_SHOULDER];
    const MockLandmark& ls = lm[LEFT_SHOULDER];

    bool crossed = rw.x < ls.x && lw.x > rs.x;

    float avg_shoulder_y = (rs.y + ls.y) / 2.0f;
    float avg_wrist_y = (rw.y + lw.y) / 2.0f;
    bool correct_height = std::abs(avg_wrist_y - avg_shoulder_y) < SIMILAR_HEIGHT_THRESHOLD;

    float wrist_dist = euclidean(rw, lw);
    bool near_chest = wrist_dist < WRIST_DIST_THRESHOLD;

    return crossed && correct_height && near_chest;
}

bool PostureDetector::is_wrist_touching_nose(const std::vector<MockLandmark>& lm, const std::string& side, float threshold) const {
    if (threshold < 0.0f) {
        threshold = NOSE_TOUCH_THRESHOLD;
    }

    const MockLandmark& nose = lm[NOSE];
    const MockLandmark& wrist = (side == "right") ? lm[RIGHT_WRIST] : lm[LEFT_WRIST];

    return euclidean(nose, wrist) < threshold;
}

} // namespace posture_game

