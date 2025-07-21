import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool, Float32MultiArray, Float32
from posture_game.postures import PostureDetector
import time
import cv2
import mediapipe as mp
import threading
#from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Lock



class CheckerNode(Node):

    def __init__(self):
        super().__init__('checker_node')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.detector = PostureDetector()
        self.last_frame_timestamp = None
        self.callback_group = ReentrantCallbackGroup()
        self.lock_presence = Lock()
        self.lock_validation = Lock()


        mp_holistic = mp.solutions.holistic

        self.holistic_presence = mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
            )

        self.holistic_validate = mp_holistic.Holistic(
            static_image_mode=False,
            model_complexity=1,
            smooth_landmarks=True,
            enable_segmentation=False,
            refine_face_landmarks=False,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
            )



        

        # Subscripciones
        self.create_subscription(Float32MultiArray,'/current_pose_data',self.validate_pose_callback,10,callback_group=self.callback_group)
        self.create_subscription(Image,'/image_raw',self.frame_callback,10,callback_group=self.callback_group)




        # Publicaciones
        self.result_publisher = self.create_publisher(Bool, '/pose_result', 10)
        self.duration_pub = self.create_publisher(Float32, '/pose_duration', 10)
        self.presence_pub = self.create_publisher(Bool, '/player_present', 10)


        # Variables de validación
        self.validation_thread = None
        self.stop_event = threading.Event()

        #Timer
        self.presence_timer = self.create_timer(0.5, self.check_presence)




    def check_presence(self):
        if self.latest_frame is None:
            return

        try:
            frame_cv2 = self.bridge.imgmsg_to_cv2(self.latest_frame)
            bgr = cv2.cvtColor(frame_cv2, cv2.COLOR_YUV2BGR_YUY2)
            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

            with self.lock_presence:
                results = self.holistic_presence.process(rgb)

            msg_out = Bool()
            msg_out.data = results.pose_landmarks is not None
            self.presence_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().warn(f"[Presencia] Error procesando frame: {e}")

    def frame_callback(self, msg):
        self.latest_frame = msg
        #self.last_frame_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def validate_pose_callback(self, msg):
        if self.latest_frame is None:
            self.get_logger().warn("⚠️ No hay imagen recibida todavía")
            return

        try:
            pose_id = int(msg.data[0])
            timeout = float(msg.data[1])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f"❌ Datos inválidos: {msg.data} - Error: {str(e)}")
            return

        self.get_logger().info(f"🧠 Iniciando validación de postura {pose_id} durante {timeout} segundos")

        # Detener cualquier validación anterior
        if self.validation_thread and self.validation_thread.is_alive():
            self.stop_event.set()
            self.validation_thread.join()

        # Iniciar nueva validación
        self.stop_event.clear()
        self.validation_thread = threading.Thread(
            target=self.run_validation_loop,
            args=(pose_id, timeout),
            daemon=True
        )
        self.validation_thread.start()

    def run_validation_loop(self, pose_id, timeout):


        start_time = time.time()
        
        # 🛑 Esperar que llegue un frame nuevo
        #initial_ts = self.last_frame_timestamp
        #wait_limit = 2.0  # segundos máximos de espera
        #while self.last_frame_timestamp == initial_ts:
            #if time.time() - start_time > wait_limit:
               # self.get_logger().warn("⏱️ No llegó un nuevo frame para iniciar validación")
                #self.publish_result(False, 0.0)
               # return
          #  time.sleep(0.05)

        while not self.stop_event.is_set():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().info("⏱️ Tiempo de validación finalizado sin éxito")
                self.publish_result(False, elapsed)
                return

            frame = self.latest_frame
            if frame is None:
                continue

            try:
                frame_cv2 = self.bridge.imgmsg_to_cv2(frame)
                if frame_cv2 is None or frame_cv2.size == 0:
                    self.get_logger().warn("[Checker] Frame vacío o inválido")
                    continue


                bgr = cv2.cvtColor(frame_cv2, cv2.COLOR_YUV2BGR_YUY2)
                rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            except Exception as e:
                self.get_logger().error(f"[ERROR] Conversión de imagen: {e}")
                continue
            
            with self.lock_validation:
                results = self.holistic_validate.process(rgb)

            
            pose_ok = False
            hand_ok = False

            if results.pose_landmarks:
                lm = results.pose_landmarks.landmark

                if pose_id == 0:
                    pose_ok = self.detector.is_right_arm_up(lm)
                elif pose_id == 1:
                    pose_ok = self.detector.is_left_arm_up(lm)
                elif pose_id == 2:
                    pose_ok = self.detector.are_arms_in_x(lm)
                elif pose_id == 6:
                    pose_ok = self.detector.is_right_arm_horizontal(lm)
                elif pose_id == 7:
                    pose_ok = self.detector.is_left_arm_horizontal(lm)
                elif pose_id == 8:
                    pose_ok = self.detector.are_both_arms_horizontal(lm)
                elif pose_id == 9:
                    pose_ok = self.detector.is_wrist_touching_nose(lm, "right") or self.detector.is_wrist_touching_nose(lm, "left")

            # Puño derecho solamente
            if pose_id == 3 and results.right_hand_landmarks:
                hand_ok = self.detector.is_right_fist_exclusive(
                    results.right_hand_landmarks,
                    results.left_hand_landmarks if results.left_hand_landmarks else None
                )

            # Puño izquierdo solamente
            elif pose_id == 4 and results.left_hand_landmarks:
                hand_ok = self.detector.is_left_fist_exclusive(
                    results.left_hand_landmarks,
                    results.right_hand_landmarks if results.right_hand_landmarks else None
                )

            # Ambos puños
            elif pose_id == 5 and results.right_hand_landmarks and results.left_hand_landmarks:
                hand_ok = self.detector.are_both_fists_closed(
                    results.right_hand_landmarks,
                    results.left_hand_landmarks
                )

            #Verificar validacion
            if pose_ok or hand_ok:
                self.get_logger().info("✅ Postura detectada correctamente")
                duration = time.time() - start_time
                self.publish_result(True, duration)
                return

            time.sleep(0.1)  # evita uso excesivo de CPU

    def publish_result(self, result: bool, duration: float):
        result_msg = Bool()
        result_msg.data = result
        self.result_publisher.publish(result_msg)

        duration_msg = Float32()
        duration_msg.data = duration
        self.duration_pub.publish(duration_msg)

        estado = "✅ Correcto" if result else "❌ Incorrecto"
        self.get_logger().info(f"📤 Resultado publicado: {estado} - Tiempo: {duration:.2f}s")
        self.get_logger().info(f"-------------------------------------------------")

def main(args=None):
    rclpy.init(args=args)
    node = CheckerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.holistic_presence.close()
        node.holistic_validate.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()