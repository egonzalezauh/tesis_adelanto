#ifndef POSTURE_GAME_POSTURES_HPP_
#define POSTURE_GAME_POSTURES_HPP_

#include <vector>
#include <cmath>
#include <string>

namespace posture_game
{

struct MockLandmark
{
    float x;
    float y;
    float z;
    float visibility;

    MockLandmark(float x_val, float y_val, float z_val, float vis = 1.0f)
        : x(x_val), y(y_val), z(z_val), visibility(vis) {}
};

inline std::vector<MockLandmark> parse_landmarks(const std::vector<float> &flat_array)
{
    std::vector<MockLandmark> landmarks;
    for (size_t i = 0; i + 2 < flat_array.size(); i += 3)
    {
        landmarks.emplace_back(flat_array[i], flat_array[i + 1], flat_array[i + 2]);
    }
    return landmarks;
}

// =============================================
// Clase PostureDetector
// =============================================

class PostureDetector
    {
    public:
        // Constructor
        PostureDetector() {}

        // Índices
        const int NOSE = 0;
        const int LEFT_SHOULDER = 11;
        const int RIGHT_SHOULDER = 12;
        const int LEFT_ELBOW = 13;
        const int RIGHT_ELBOW = 14;
        const int LEFT_WRIST = 15;
        const int RIGHT_WRIST = 16;

        // Umbrales (thresholds)
        const float ANGLE_THRESHOLD = 160.0f;
        const float VERTICAL_THRESHOLD = 0.15f;
        const float HORIZONTAL_THRESHOLD = 0.08f;
        const float VERTICAL_DIFF_THRESHOLD = 0.07f;
        const float WRIST_DIST_THRESHOLD = 0.1f;
        const float SIMILAR_HEIGHT_THRESHOLD = 0.08f;
        const float NOSE_TOUCH_THRESHOLD = 0.10f;

        // Distancia euclidiana
        float euclidean(const MockLandmark &p1, const MockLandmark &p2) const
        {
            float dx = p1.x - p2.x;
            float dy = p1.y - p2.y;
            return std::sqrt(dx * dx + dy * dy);
        }

        // Verifica si el puño está cerrado
        bool is_fist_closed(const std::vector<MockLandmark> &lm) const
        {
            std::vector<int> tips = {8, 12, 16, 20};
            std::vector<int> pips = {6, 10, 14, 18};
            int base = 0;
            int bent_fingers = 0;

            const MockLandmark &base_lm = lm[base];
            for (size_t i = 0; i < tips.size(); ++i)
            {
                const MockLandmark &tip = lm[tips[i]];
                const MockLandmark &pip = lm[pips[i]];
                if (euclidean(tip, base_lm) < euclidean(pip, base_lm))
                {
                    ++bent_fingers;
                }
            }

            const MockLandmark &thumb_tip = lm[4];
            const MockLandmark &thumb_ip = lm[3];
            float thumb_dist = euclidean(thumb_tip, thumb_ip);

            return bent_fingers >= 3 && thumb_dist < 0.05f;
        }

        // Solo el puño derecho está cerrado
        bool is_right_fist_exclusive(const std::vector<MockLandmark> &right_hand,
                                    const std::vector<MockLandmark> *left_hand = nullptr) const
        {
            bool right_closed = is_fist_closed(right_hand);
            bool left_closed = left_hand ? is_fist_closed(*left_hand) : false;

            return right_closed && !left_closed;
        }

        // Solo el puño izquierdo está cerrado
        bool is_left_fist_exclusive(const std::vector<MockLandmark> &left_hand,
                                    const std::vector<MockLandmark> *right_hand = nullptr) const
        {
            bool left_closed = is_fist_closed(left_hand);
            bool right_closed = right_hand ? is_fist_closed(*right_hand) : false;

            return left_closed && !right_closed;
        }

        // Ambos puños están cerrados
        bool are_both_fists_closed(const std::vector<MockLandmark> &right_hand,
                                    const std::vector<MockLandmark> &left_hand) const
        {
            return is_fist_closed(right_hand) && is_fist_closed(left_hand);
        }


        // Brazo derecho horizontal
        bool is_right_arm_horizontal(const std::vector<MockLandmark> &lm) const
        {
            const MockLandmark &r_shoulder = lm[RIGHT_SHOULDER];
            const MockLandmark &r_elbow = lm[RIGHT_ELBOW];
            const MockLandmark &r_wrist = lm[RIGHT_WRIST];

            const MockLandmark &l_shoulder = lm[LEFT_SHOULDER];
            const MockLandmark &l_elbow = lm[LEFT_ELBOW];
            const MockLandmark &l_wrist = lm[LEFT_WRIST];

            std::vector<float> v1 = {r_shoulder.x - r_elbow.x, r_shoulder.y - r_elbow.y};
            std::vector<float> v2 = {r_wrist.x - r_elbow.x, r_wrist.y - r_elbow.y};

            float dot = v1[0]*v2[0] + v1[1]*v2[1];
            float norm_v1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
            float norm_v2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);

            float cos_angle = dot / (norm_v1 * norm_v2);
            cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));  // clamp manual
            float angle = std::acos(cos_angle) * (180.0f / M_PI);

            bool altura = std::abs(r_shoulder.y - r_wrist.y) < 0.05f;
            bool brazo_derecho_ok = angle > ANGLE_THRESHOLD && altura;

            bool brazo_izquierdo_abajo = std::abs(l_shoulder.y - l_wrist.y) > 0.12f ||
                                        std::abs(l_elbow.y - l_wrist.y) < 0.05f;

            return brazo_derecho_ok && brazo_izquierdo_abajo;
        }



        // Brazo izquierdo horizontal
        bool is_left_arm_horizontal(const std::vector<MockLandmark> &lm) const
        {
            const MockLandmark &l_shoulder = lm[LEFT_SHOULDER];
            const MockLandmark &l_elbow = lm[LEFT_ELBOW];
            const MockLandmark &l_wrist = lm[LEFT_WRIST];

            const MockLandmark &r_shoulder = lm[RIGHT_SHOULDER];
            const MockLandmark &r_elbow = lm[RIGHT_ELBOW];
            const MockLandmark &r_wrist = lm[RIGHT_WRIST];

            std::vector<float> v1 = {l_shoulder.x - l_elbow.x, l_shoulder.y - l_elbow.y};
            std::vector<float> v2 = {l_wrist.x - l_elbow.x, l_wrist.y - l_elbow.y};

            float dot = v1[0]*v2[0] + v1[1]*v2[1];
            float norm_v1 = std::sqrt(v1[0]*v1[0] + v1[1]*v1[1]);
            float norm_v2 = std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);

            float cos_angle = dot / (norm_v1 * norm_v2);
            cos_angle = std::max(-1.0f, std::min(1.0f, cos_angle));  // clamp manual
            float angle = std::acos(cos_angle) * (180.0f / M_PI);

            bool altura = std::abs(l_shoulder.y - l_wrist.y) < 0.05f;
            bool brazo_izquierdo_ok = angle > ANGLE_THRESHOLD && altura;

            bool brazo_derecho_abajo = std::abs(r_shoulder.y - r_wrist.y) > 0.12f ||
                                        std::abs(r_elbow.y - r_wrist.y) < 0.05f;

            return brazo_izquierdo_ok && brazo_derecho_abajo;
        }

        // Ambos brazos horizontales
        bool are_both_arms_horizontal(const std::vector<MockLandmark> &lm) const
        {
            return is_right_arm_horizontal(lm) && is_left_arm_horizontal(lm);
        }
        
        // Brazo derecho arriba 
        bool is_right_arm_up(const std::vector<MockLandmark> &lm) const
        {
            const MockLandmark &r_shoulder = lm[RIGHT_SHOULDER];
            const MockLandmark &r_elbow = lm[RIGHT_ELBOW];
            const MockLandmark &r_wrist = lm[RIGHT_WRIST];

            const MockLandmark &l_shoulder = lm[LEFT_SHOULDER];
            const MockLandmark &l_elbow = lm[LEFT_ELBOW];
            const MockLandmark &l_wrist = lm[LEFT_WRIST];

            bool right_vertical = (r_shoulder.y - r_wrist.y) > VERTICAL_THRESHOLD;
            bool right_horizontal = std::abs(r_wrist.x - r_shoulder.x) < HORIZONTAL_THRESHOLD;
            bool right_straight = r_wrist.y < r_elbow.y && r_elbow.y < r_shoulder.y;

            bool left_arm_down = l_wrist.y > l_shoulder.y || l_elbow.y > l_shoulder.y;

            return right_vertical && right_horizontal && right_straight && left_arm_down;
        }
        
        //Brazo izquierdo arriba
        bool is_left_arm_up(const std::vector<MockLandmark> &lm) const
        {
            const MockLandmark &l_shoulder = lm[LEFT_SHOULDER];
            const MockLandmark &l_elbow = lm[LEFT_ELBOW];
            const MockLandmark &l_wrist = lm[LEFT_WRIST];

            const MockLandmark &r_shoulder = lm[RIGHT_SHOULDER];
            const MockLandmark &r_elbow = lm[RIGHT_ELBOW];
            const MockLandmark &r_wrist = lm[RIGHT_WRIST];

            bool left_vertical = (l_shoulder.y - l_wrist.y) > VERTICAL_THRESHOLD;
            bool left_horizontal = std::abs(l_wrist.x - l_shoulder.x) < HORIZONTAL_THRESHOLD;
            bool left_straight = l_wrist.y < l_elbow.y && l_elbow.y < l_shoulder.y;

            bool right_arm_down = r_wrist.y > r_shoulder.y || r_elbow.y > r_shoulder.y;

            return left_vertical && left_horizontal && left_straight && right_arm_down;
        }


        // Brazos en X
        bool are_arms_in_x(const std::vector<MockLandmark> &lm) const
        {
            const MockLandmark &rw = lm[RIGHT_WRIST];
            const MockLandmark &lw = lm[LEFT_WRIST];
            const MockLandmark &rs = lm[RIGHT_SHOULDER];
            const MockLandmark &ls = lm[LEFT_SHOULDER];

            bool crossed = rw.x < ls.x && lw.x > rs.x;

            float avg_shoulder_y = (rs.y + ls.y) / 2.0f;
            float avg_wrist_y = (rw.y + lw.y) / 2.0f;
            bool correct_height = std::abs(avg_wrist_y - avg_shoulder_y) < SIMILAR_HEIGHT_THRESHOLD;

            float wrist_dist = euclidean(rw, lw);
            bool near_chest = wrist_dist < WRIST_DIST_THRESHOLD;

            return crossed && correct_height && near_chest;
        }

        bool is_wrist_touching_nose(const std::vector<MockLandmark> &lm, const std::string &side = "right", float threshold = -1.0f) const
        {
            if (threshold < 0.0f)
            {
                threshold = NOSE_TOUCH_THRESHOLD;
            }

            const MockLandmark &nose = lm[NOSE];
            const MockLandmark &wrist = (side == "right") ? lm[RIGHT_WRIST] : lm[LEFT_WRIST];

            return euclidean(nose, wrist) < threshold;
        }

    };

} //namespace

#endif  // POSTURE_GAME_POSTURES_HPP_
