import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, Float32
from posture_game.postures import PostureDetector, parse_landmarks
from posture_game_interfaces.msg import HolisticLandmarks
import time
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CheckerNode(Node):

    def __init__(self):
        super().__init__('checker_node')


        self.detector = PostureDetector()
        self.callback_group = ReentrantCallbackGroup()




        #Listas de landarmaks de cada cuerpo
        self.current_pose_landmarks = []
        self.current_left_hand_landmarks = []
        self.current_right_hand_landmarks = []

        # Subscripciones
        self.create_subscription(Float32MultiArray,'/current_pose_data',self.validate_pose_callback,10,callback_group=self.callback_group)
        self.create_subscription(HolisticLandmarks,'/holistic_landmarks',self.landmarks_callback,10,callback_group=self.callback_group)

        # Publicaciones
        self.result_publisher = self.create_publisher(Bool, '/pose_result', 10)
        self.duration_pub = self.create_publisher(Float32, '/pose_duration', 10)
        self.presence_pub = self.create_publisher(Bool, '/player_present', 10)

        # Variables de validación
        self.validation_thread = None
        self.stop_event = threading.Event()

        #Timer
        self.presence_timer = self.create_timer(0.5, self.check_presence)


    def landmarks_callback(self, msg: HolisticLandmarks):
        self.current_pose_landmarks = parse_landmarks(msg.pose_landmarks)
        self.current_left_hand_landmarks = parse_landmarks(msg.left_hand_landmarks)
        self.current_right_hand_landmarks = parse_landmarks(msg.right_hand_landmarks)


    def check_presence(self):
        is_present = len(self.current_pose_landmarks) > 0
        self.presence_pub.publish(Bool(data=is_present))


    def validate_pose_callback(self, msg):

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
        

        while not self.stop_event.is_set():
            elapsed = time.time() - start_time
            if elapsed > timeout:
                self.get_logger().info("⏱️ Tiempo de validación finalizado sin éxito")
                self.publish_result(False, elapsed)
                return

            lm = self.current_pose_landmarks
            rh = self.current_right_hand_landmarks
            lh = self.current_left_hand_landmarks

            
            pose_ok = False
            hand_ok = False

            if lm:
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
            if pose_id == 3 and rh:
                hand_ok = self.detector.is_right_fist_exclusive(rh,lh if lh else None)

            # Puño izquierdo solamente
            elif pose_id == 4 and lh:
                hand_ok = self.detector.is_left_fist_exclusive(lh,rh if rh else None)

            # Ambos puños
            elif pose_id == 5 and rh and lh:
                hand_ok = self.detector.are_both_fists_closed(rh,lh)

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
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()