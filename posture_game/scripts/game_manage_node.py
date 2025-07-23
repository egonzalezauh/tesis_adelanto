import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Float32, String, Int16
import random
import time
import statistics
from posture_game.message import MessageManager
from posture_game.stats_manager import StatsManager
from collections import Counter
import threading

class GameManager(Node):
    def __init__(self):
        super().__init__('posture_game_node')
        
        self.get_logger().info("🕹️ Juego de memoria con posturas iniciado")

        # Configuraciones
        self.TIME_LIMIT = 5
        self.MAX_ATTEMPTS = 3
        self.POSTURES = [
            "brazo derecho arriba", "brazo izquierdo bien alto", "brazos en x", "puño derecho", "puño izquierdo",
            "ambos puños", "brazo derecho horizontal", "brazo izquierdo horizontal", "ambos brazos horizontales", "muñeca en nariz"
        ]

        # Estado del juego
        self.sequence = []
        self.score = 0
        self.total_fails = 0
        self.success = True
        self.start_time = time.time()
        self.response_times = []
        self.detailed_attempts = []
        self.emotions_this_attempt = [] 
        self.errors_by_pose = [0 for _ in self.POSTURES]
        self.first_error_level = None
        self.emotion_counts = {"Angry": 0,"Disgust": 0,"Fear": 0,"Happy": 0,"Sad": 0,"Surprise": 0,"Neutral": 0}
        self.game_active = True
        self.last_help_shown = 0
        self.speak_pose_lock = threading.Lock()


        # Publicaciones
        self.pose_pub = self.create_publisher(Float32MultiArray, '/current_pose_data', 10) #Publica el ID y el tiempo 
        self.voice_pub = self.create_publisher(String, '/game_feedback', 10) #Publica el string a reproducir
        self.shutdown_all_pub = self.create_publisher(Bool, '/shutdown_all', 10) #Publica que el juego se ha acabado para apagar otros nodos
        self.visual_pub = self.create_publisher(String, '/yaren_visual_state', 10)




        #Subscripciones
        self.EMOTIONS_LIST = ["Angry", "Disgust", "Fear", "Happy", "Sad", "Surprise", "Neutral"]
        self.emotions_sub = self.create_subscription(Int16, '/emotion', self.emotion_callback, 10)

        self.duration_sub = self.create_subscription(Float32, '/pose_duration', self.duration_callback, 10) #Recibe la duracion de la pose
        self.duration_received = None

        self.result_sub = self.create_subscription(Bool, '/pose_result', self.result_callback, 10) #Recibe el resultado de la pose
        self.result_received = None

        self.audio_on_sub = self.create_subscription(Bool, '/audio_playing', self.audio_status_callback, 10) #Recibe el status del TTS 
        self.audio_playing = False

        self.presence_sub = self.create_subscription(Bool, '/player_present', self.presence_callback, 10)
        self.person_present = False  # Estado actual


        # Utilidades
        self.messages = MessageManager()
        self.stats_manager = StatsManager()




    def audio_status_callback(self, msg):
        self.audio_playing = msg.data  #True cuando habla, False cuando termina

    def result_callback(self, msg):
        self.result_received = msg.data

    def duration_callback(self, msg):
        self.duration_received = msg.data

    def presence_callback(self, msg):
        self.person_present = msg.data

    def emotion_callback(self, msg):
        try:
            emotion_id = msg.data
            if 0 <= emotion_id < len(self.EMOTIONS_LIST):
                label = self.EMOTIONS_LIST[emotion_id]
                self.emotion_counts[label] += 1  #acumulador general 
                self.emotions_this_attempt.append(label) #solo para un intento
        except Exception as e:
            self.get_logger().warn(f"[Emotions] Error: {e}")

    def show_message(self, msg):

        #Reiniciar estado antes de hablar para evitar race condition logico
        self.audio_playing = False

        #Publicar mensaje
        tts_msg = String()
        tts_msg.data = msg
        self.voice_pub.publish(tts_msg)

        #Esperar a que comience a hablar (máx. 2 segundos)
        timeout_start = time.time()
        while not self.audio_playing:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - timeout_start > 2:
                break  # No comenzó a hablar

        #Esperar a que termine de hablar (máx. 10 segundos)
        wait_time = 0.0
        while self.audio_playing:
            rclpy.spin_once(self, timeout_sec=0.1)
            wait_time += 0.1
            if wait_time > 10.0:
                break

    def speak_and_pose(self, text: str, pose: str):
        with self.speak_pose_lock:
            def hablar():
                tts_msg = String()
                tts_msg.data = text
                self.voice_pub.publish(tts_msg)

            def mover():
                pose_msg = String()
                pose_msg.data = pose.lower()
                self.visual_pub.publish(pose_msg)
                self.get_logger().info(f"🦾 Postura enviada a Yaren: {pose}")
                time.sleep(1.0)  # dejar que se mueva visualmente

            t1 = threading.Thread(target=hablar)
            t2 = threading.Thread(target=mover)

            t1.start()
            t2.start()
            t1.join()
            t2.join()

            time.sleep(0.5)  # ❗ Esto da un pequeño respiro antes de otra ejecución


    def play(self):

        # -------------------------------------------------------COMENTA/ELIMINA ESTO POCHI-----------------------------------------------
        presence_counter = 0
        required_count = 50  # 50 ciclos de 0.1s = 3 segundos

        #Verificar si hay una persona adelante
        while presence_counter < required_count:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.person_present:
                presence_counter += 1
            else:
                presence_counter = 0  # Reinicia si desaparece
        # -------------------------------------------------------COMENTA/ELIMINA ESTO POCHI-----------------------------------------------
        
        name = "Erick"


        self.speak_and_pose("Bienvenido al juego de las posturas", "welcome")
        self.show_message("Memoriza la secuencia y haz las posturas en ese mismo orden")

        while self.game_active:
            self.show_message(f" Nivel {len(self.sequence) + 1}")

            if self.success or len(self.sequence) == 0:
                while True:
                    new_pose = random.choice(range(len(self.POSTURES)))
                    if len(self.sequence) == 0 or new_pose != self.sequence[-1]:
                        break
                full_sequence = self.sequence + [new_pose]

# ---------------------------------------------------------------------------------------------------------------
            # Determinar ayuda si aplica
            if self.total_fails == 1:
                if self.last_help_shown < 1:
                    self.show_message("Ayuda 1, te repito la misma secuencia")
                    self.last_help_shown = 1

                timeout = self.TIME_LIMIT
                sequence_now = full_sequence

            elif self.total_fails == 2:
                if self.last_help_shown < 2:
                    self.show_message("Ayuda 2, te doy más tiempo")
                    self.last_help_shown = 2

                timeout = self.TIME_LIMIT + 5
                sequence_now = full_sequence


            elif self.total_fails == 3:
                if self.last_help_shown < 3:
                    self.show_message("Ayuda 3, reduzcamos una postura de la secuencia")
                    self.last_help_shown = 3

                timeout = self.TIME_LIMIT
                sequence_now = full_sequence[:-1]

            else:
                timeout = self.TIME_LIMIT
                sequence_now = full_sequence

            self.success = True
# ---------------------------------------------------------------------------------------------------------------
            # Mostrar la secuencia completa
            for pose_id in sequence_now:
                self.show_message(f" {self.POSTURES[pose_id].upper()}")

            self.show_message("Ahora tu repite la secuencia")

            

# ---------------------------------------------------------------------------------------------------------------
            # Validar postura por postura
            for idx, expected in enumerate(sequence_now):
                self.emotions_this_attempt = []  # limpiar antes de cada intento
                self.result_received = None
                self.duration_received = None

                msg = Float32MultiArray()
                msg.data = [float(expected), float(timeout)]

                
                self.pose_pub.publish(msg)
                print(f"🟢 Publicado: Pose {expected} | Timeout: {timeout}s")

                
                while self.result_received is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    print("Revisando.......")

                result = self.result_received

                while self.duration_received is None:
                    rclpy.spin_once(self, timeout_sec=0.1)
                    


                if result:
                    self.response_times.append(self.duration_received)

                    # Analizar emociones detectadas durante este intento
                    if self.emotions_this_attempt:
                        from collections import Counter
                        counter = Counter(self.emotions_this_attempt)
                        predominant_emotion = counter.most_common(1)[0][0]
                    else:
                        predominant_emotion = "No detectada"

                    # Guardar intento detallado con postura, tiempo y emoción
                    self.detailed_attempts.append({
                        "Postura": self.POSTURES[expected],
                        "Tiempo de respuesta (s)": round(self.duration_received, 2),
                        "Emoción predominante": predominant_emotion
                    })


                else:
                    self.success = False
                    self.errors_by_pose[expected] += 1

                    if self.first_error_level is None:
                        self.first_error_level = len(full_sequence)

                    if self.total_fails < self.MAX_ATTEMPTS:
                        self.speak_and_pose(random.choice(self.messages.error_messages),"error")

                    
                    break  # Se rompe la secuencia tras el primer error
# ---------------------------------------------------------------------------------------------------------------

            #Determinar que hacer despues del acierto o error
            if self.success:
                self.sequence = full_sequence
                self.score += 1
                self.speak_and_pose(random.choice(self.messages.success_messages),"success")

            else:
                self.total_fails += 1
                if self.total_fails > self.MAX_ATTEMPTS:
                    self.end_game(name)
                    self.game_active = False
                else:
                    rclpy.spin_once(self, timeout_sec=1)

#----------------------------------------------------------------------------------------------------------------
    def end_game(self, name):
        self.show_message(f" Fin del juego, {name}. Puntaje final: {self.score}")
        
        self.speak_and_pose(random.choice(self.messages.final_messages),"end")

        total_time = time.time() - self.start_time
        avg_time = sum(self.response_times) / len(self.response_times) if self.response_times else 0
        std_dev = statistics.stdev(self.response_times) if len(self.response_times) > 1 else 0
        total_attempts = len(self.response_times) + sum(self.errors_by_pose)

        error_stats = {self.POSTURES[i]: self.errors_by_pose[i] for i in range(len(self.POSTURES))}

        self.stats_manager.save_stats(
            name=name,
            score=self.score,
            emotions=self.emotion_counts,
            total_duration=total_time,
            avg_response_time=avg_time,
            response_times=self.response_times,
            response_deviation=std_dev,
            total_attempts=total_attempts,
            errors_per_posture=error_stats,
            first_error_level=self.first_error_level,
            detailed_attempts=self.detailed_attempts
        )
        self.shutdown_all_pub.publish(Bool(data=True))
        self.get_logger().info("🛑 Señal de apagado general enviada (/shutdown_all)")




def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    node.play()
    node.destroy_node()
    rclpy.shutdown()

