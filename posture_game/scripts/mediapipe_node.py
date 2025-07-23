#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from posture_game_interfaces.msg import HolisticLandmarks
from cv_bridge import CvBridge
import mediapipe as mp
import cv2

class MediaPipeNode(Node):
    def __init__(self):
        super().__init__('mediapipe_node')

        self.bridge = CvBridge()

        # Crear publisher del mensaje personalizado
        self.publisher = self.create_publisher(HolisticLandmarks, '/holistic_landmarks', 10)

        # Subscribirse a /image_raw
        self.subscription = self.create_subscription(Image,'/image_raw',self.image_callback,10)

        # Inicializar MediaPipe Holistic
        mp_holistic = mp.solutions.holistic
        self.holistic = mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )

        self.get_logger().info("MediaPipe Node iniciado correctamente ✅")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg)
            bgr = cv2.cvtColor(frame, cv2.COLOR_YUV2BGR_YUY2)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            results = self.holistic.process(rgb)

            pose = results.pose_landmarks.landmark if results.pose_landmarks else []
            left_hand = results.left_hand_landmarks.landmark if results.left_hand_landmarks else []
            right_hand = results.right_hand_landmarks.landmark if results.right_hand_landmarks else []

            flat_pose = [coord for lm in pose for coord in (lm.x, lm.y, lm.z)]
            flat_left = [coord for lm in left_hand for coord in (lm.x, lm.y, lm.z)]
            flat_right = [coord for lm in right_hand for coord in (lm.x, lm.y, lm.z)]

            msg_out = HolisticLandmarks()
            msg_out.header.stamp = self.get_clock().now().to_msg()
            msg_out.pose_landmarks = flat_pose
            msg_out.left_hand_landmarks = flat_left
            msg_out.right_hand_landmarks = flat_right

            self.publisher.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Error en MediaPipeNode: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MediaPipeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.holistic.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
