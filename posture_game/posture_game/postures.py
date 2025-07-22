import numpy as np
from typing import List


# ---------------------------------------------
# Clase y función auxiliar para convertir arrays planos en objetos
# ---------------------------------------------

class MockLandmark:
    def __init__(self, x, y, z, visibility=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.visibility = visibility

def parse_landmarks(flat_array: List[float]) -> List["MockLandmark"]:
        landmarks = []
        for i in range(0, len(flat_array), 3):
            x = flat_array[i]
            y = flat_array[i + 1]
            z = flat_array[i + 2]
            landmarks.append(MockLandmark(x, y, z))
        return landmarks


# ---------------------------------------------
# Detector de Posturas (independiente de MediaPipe)
# ---------------------------------------------


class PostureDetector:
    def __init__(self):


        # Índices equivalentes a MediaPipe
        self.NOSE = 0
        self.LEFT_SHOULDER = 11
        self.RIGHT_SHOULDER = 12
        self.LEFT_ELBOW = 13
        self.RIGHT_ELBOW = 14
        self.LEFT_WRIST = 15
        self.RIGHT_WRIST = 16

        # Thresholds
        self.ANGLE_THRESHOLD = 160
        self.VERTICAL_THRESHOLD = 0.15
        self.HORIZONTAL_THRESHOLD = 0.08
        self.VERTICAL_DIFF_THRESHOLD = 0.07
        self.WRIST_DIST_THRESHOLD = 0.1
        self.SIMILAR_HEIGHT_THRESHOLD = 0.08
        self.NOSE_TOUCH_THRESHOLD = 0.10

    def _euclidean(self, p1, p2):
        return np.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

    def is_fist_closed(self, lm):
        tips = [8, 12, 16, 20]
        pips = [6, 10, 14, 18]
        base = 0
        bent_fingers = 0

        base_lm = lm[base]
        for tip_idx, pip_idx in zip(tips, pips):
            tip = lm[tip_idx]
            pip = lm[pip_idx]
            if self._euclidean(tip, base_lm) < self._euclidean(pip, base_lm):
                bent_fingers += 1

        thumb_tip = lm[4]
        thumb_ip = lm[3]
        thumb_dist = self._euclidean(thumb_tip, thumb_ip)
        return bent_fingers >= 3 and thumb_dist < 0.05
    
    def is_right_fist_exclusive(self, right_hand, left_hand=None):
        right_closed = self.is_fist_closed(right_hand)
        left_closed = self.is_fist_closed(left_hand) if left_hand else False

        return right_closed and not left_closed
    
    def is_left_fist_exclusive(self, left_hand, right_hand=None):
        left_closed = self.is_fist_closed(left_hand)
        right_closed = self.is_fist_closed(right_hand) if right_hand else False

        return left_closed and not right_closed

    def are_both_fists_closed(self, right_hand, left_hand):
        return self.is_fist_closed(right_hand) and self.is_fist_closed(left_hand)


    def is_right_arm_horizontal(self, lm):
        r_shoulder = lm[self.RIGHT_SHOULDER]
        r_elbow = lm[self.RIGHT_ELBOW]
        r_wrist = lm[self.RIGHT_WRIST]

        l_shoulder = lm[self.LEFT_SHOULDER]
        l_elbow = lm[self.LEFT_ELBOW]
        l_wrist = lm[self.LEFT_WRIST]
        # Validar brazo derecho estirado y horizontal
        v1 = np.array([r_shoulder.x - r_elbow.x, r_shoulder.y - r_elbow.y])
        v2 = np.array([r_wrist.x - r_elbow.x, r_wrist.y - r_elbow.y])
        angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) /
                            (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
        altura = abs(r_shoulder.y - r_wrist.y) < 0.05
        brazo_derecho_ok = angle > self.ANGLE_THRESHOLD and altura

        # Validar que el brazo izquierdo NO esté horizontal
        brazo_izquierdo_abajo = abs(l_shoulder.y - l_wrist.y) > 0.12 or \
                                abs(l_elbow.y - l_wrist.y) < 0.05

        return brazo_derecho_ok and brazo_izquierdo_abajo

    def is_left_arm_horizontal(self, lm):
        l_shoulder = lm[self.LEFT_SHOULDER]
        l_elbow = lm[self.LEFT_ELBOW]
        l_wrist = lm[self.LEFT_WRIST]

        r_shoulder = lm[self.RIGHT_SHOULDER]
        r_elbow = lm[self.RIGHT_ELBOW]
        r_wrist = lm[self.RIGHT_WRIST]

        # Validar brazo izquierdo estirado y horizontal
        v1 = np.array([l_shoulder.x - l_elbow.x, l_shoulder.y - l_elbow.y])
        v2 = np.array([l_wrist.x - l_elbow.x, l_wrist.y - l_elbow.y])
        angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) /
                            (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
        altura = abs(l_shoulder.y - l_wrist.y) < 0.05
        brazo_izquierdo_ok = angle > self.ANGLE_THRESHOLD and altura

        # Validar que el brazo derecho NO esté horizontal
        brazo_derecho_abajo = abs(r_shoulder.y - r_wrist.y) > 0.12 or \
                            abs(r_elbow.y - r_wrist.y) < 0.05

        return brazo_izquierdo_ok and brazo_derecho_abajo


    def are_both_arms_horizontal(self, landmarks):
        return self.is_right_arm_horizontal(landmarks) and self.is_left_arm_horizontal(landmarks)


    def is_right_arm_up(self, lm):
        r_shoulder = lm[self.RIGHT_SHOULDER]
        r_elbow = lm[self.RIGHT_ELBOW]
        r_wrist = lm[self.RIGHT_WRIST]

        l_shoulder = lm[self.LEFT_SHOULDER]
        l_elbow = lm[self.LEFT_ELBOW]
        l_wrist = lm[self.LEFT_WRIST]


        right_vertical = r_shoulder.y - r_wrist.y > self.VERTICAL_THRESHOLD
        right_horizontal = abs(r_wrist.x - r_shoulder.x) < self.HORIZONTAL_THRESHOLD
        right_straight = r_wrist.y < r_elbow.y < r_shoulder.y

        # Nueva verificación: el brazo izquierdo no debe estar arriba
        left_arm_down = l_wrist.y > l_shoulder.y or l_elbow.y > l_shoulder.y

        return right_vertical and right_horizontal and right_straight and left_arm_down

    def is_left_arm_up(self, lm):
        l_shoulder = lm[self.LEFT_SHOULDER]
        l_elbow = lm[self.LEFT_ELBOW]
        l_wrist = lm[self.LEFT_WRIST]

        r_shoulder = lm[self.RIGHT_SHOULDER]
        r_elbow = lm[self.RIGHT_ELBOW]
        r_wrist = lm[self.RIGHT_WRIST]

        left_vertical = l_shoulder.y - l_wrist.y > self.VERTICAL_THRESHOLD
        left_horizontal = abs(l_wrist.x - l_shoulder.x) < self.HORIZONTAL_THRESHOLD
        left_straight = l_wrist.y < l_elbow.y < l_shoulder.y

        right_arm_down = r_wrist.y > r_shoulder.y or r_elbow.y > r_shoulder.y

        return left_vertical and left_horizontal and left_straight and right_arm_down
    

    def are_arms_in_x(self, lm):
        rw = lm[self.RIGHT_WRIST]
        lw = lm[self.LEFT_WRIST]
        rs = lm[self.RIGHT_SHOULDER]
        ls = lm[self.LEFT_SHOULDER]

        # 1. ¿Los brazos se cruzan?
        crossed = rw.x < ls.x and lw.x > rs.x

        # 2. ¿Están a la altura del pecho?
        avg_shoulder_y = (rs.y + ls.y) / 2
        avg_wrist_y = (rw.y + lw.y) / 2
        correct_height = abs(avg_wrist_y - avg_shoulder_y) < 0.08  # más estricto

        # 3. ¿Los puños están cerca entre sí?
        wrist_dist = self._euclidean(rw, lw)
        near_chest = wrist_dist < self.WRIST_DIST_THRESHOLD

        return crossed and correct_height and near_chest


    def is_wrist_touching_nose(self, lm, side="right", threshold=None):
        if threshold is None:
            threshold = self.NOSE_TOUCH_THRESHOLD

        nose = lm[self.NOSE]
        wrist = lm[self.RIGHT_WRIST] if side == "right" \
            else lm[self.LEFT_WRIST]
        return self._euclidean(nose, wrist) < threshold
