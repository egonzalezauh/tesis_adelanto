import mediapipe as mp
import numpy as np

class PostureDetector:
    def __init__(self):
        self.mp_holistic = mp.solutions.holistic

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

    def is_fist_closed(self, hand_landmarks):
        lm = hand_landmarks.landmark
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

    def is_arm_extended(self, landmarks, side="right"):
        if side == "right":
            shoulder = landmarks[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER.value]
            elbow = landmarks[self.mp_holistic.PoseLandmark.RIGHT_ELBOW.value]
            wrist = landmarks[self.mp_holistic.PoseLandmark.RIGHT_WRIST.value]
        else:
            shoulder = landmarks[self.mp_holistic.PoseLandmark.LEFT_SHOULDER.value]
            elbow = landmarks[self.mp_holistic.PoseLandmark.LEFT_ELBOW.value]
            wrist = landmarks[self.mp_holistic.PoseLandmark.LEFT_WRIST.value]

        v1 = np.array([shoulder.x - elbow.x, shoulder.y - elbow.y])
        v2 = np.array([wrist.x - elbow.x, wrist.y - elbow.y])
        angle = np.degrees(np.arccos(np.clip(np.dot(v1, v2) / 
                        (np.linalg.norm(v1) * np.linalg.norm(v2)), -1.0, 1.0)))
        vertical_diff = abs(shoulder.y - wrist.y)
        return angle > self.ANGLE_THRESHOLD and vertical_diff < self.VERTICAL_DIFF_THRESHOLD

    def is_right_arm_up(self, landmarks):
        shoulder = landmarks[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER.value]
        wrist = landmarks[self.mp_holistic.PoseLandmark.RIGHT_WRIST.value]
        elbow = landmarks[self.mp_holistic.PoseLandmark.RIGHT_ELBOW.value]
        vertical = shoulder.y - wrist.y > self.VERTICAL_THRESHOLD
        horizontal = abs(wrist.x - shoulder.x) < self.HORIZONTAL_THRESHOLD
        brazo_recto = wrist.y < elbow.y < shoulder.y
        return vertical and horizontal and brazo_recto

    def is_left_arm_up(self, landmarks):
        shoulder = landmarks[self.mp_holistic.PoseLandmark.LEFT_SHOULDER.value]
        wrist = landmarks[self.mp_holistic.PoseLandmark.LEFT_WRIST.value]
        elbow = landmarks[self.mp_holistic.PoseLandmark.LEFT_ELBOW.value]
        vertical = shoulder.y - wrist.y > self.VERTICAL_THRESHOLD
        horizontal = abs(wrist.x - shoulder.x) < self.HORIZONTAL_THRESHOLD
        brazo_recto = wrist.y < elbow.y < shoulder.y
        return vertical and horizontal and brazo_recto

    def are_arms_in_x(self, landmarks):
        rw = landmarks[self.mp_holistic.PoseLandmark.RIGHT_WRIST.value]
        lw = landmarks[self.mp_holistic.PoseLandmark.LEFT_WRIST.value]
        rs = landmarks[self.mp_holistic.PoseLandmark.RIGHT_SHOULDER.value]
        ls = landmarks[self.mp_holistic.PoseLandmark.LEFT_SHOULDER.value]

        crossed = rw.x < ls.x and lw.x > rs.x
        wrist_dist = self._euclidean(rw, lw)
        similar_height = abs(rw.y - lw.y) < self.SIMILAR_HEIGHT_THRESHOLD

        return crossed and wrist_dist < self.WRIST_DIST_THRESHOLD and similar_height

    def is_wrist_touching_nose(self, landmarks, side="right", threshold=None):
        if threshold is None:
            threshold = self.NOSE_TOUCH_THRESHOLD

        nose = landmarks[self.mp_holistic.PoseLandmark.NOSE.value]
        wrist = landmarks[self.mp_holistic.PoseLandmark.RIGHT_WRIST.value] if side == "right" \
            else landmarks[self.mp_holistic.PoseLandmark.LEFT_WRIST.value]
        return self._euclidean(nose, wrist) < threshold
