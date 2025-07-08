import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool, Float32, String
import random
import time
import statistics
from posture_game.message import MessageManager
from posture_game.stats_manager import StatsManager


class GameManager(Node):
    def __init__(self):
        super().__init__('posture_game_node')
        
        self.get_logger().info("🕹️ Juego de memoria con posturas iniciado")

        # Configuraciones
        self.TIME_LIMIT = 4
        self.MAX_ATTEMPTS = 3
        self.POSTURES = [
            "derecho arriba", "izquierdo arriba", "brazos en x", "puño derecho", "puño izquierdo",
            "ambos puños", "derecho horizontal", "izquierdo horizontal", "ambos brazos horizontales", "muñeca en nariz"
        ]

        # Estado del juego
        self.sequence = []
        self.score = 0
        self.total_fails = 0
        self.success = True
        self.start_time = time.time()
        self.response_times = []
        self.errors_by_pose = [0 for _ in self.POSTURES]
        self.first_error_level = None
        self.emotions = []
        self.game_active = True

        # Publicaciones
        self.pose_pub = self.create_publisher(Float32MultiArray, '/current_pose_data', 10) #Publica el ID y el tiempo 
        self.voice_pub = self.create_publisher(String, '/game_feedback', 10) #Publica el string a reproducir 


        #Subscripciones
        self.duration_pub = self.create_subscription(Float32, '/pose_duration', self.duration_callback, 10) #Recibe la duracion de la pose
        self.result_received = None
        self.result_sub = self.create_subscription(Bool, '/pose_result', self.result_callback, 10) #Recibe el resultado de la pose
        self.duration_received = None
        self.audio_on_sub = self.create_subscription(Bool, '/audio_playing', self.audio_status_callback, 10) #Recibe el status del TTS 
        self.audio_playing = False
        
        # Utilidades
        self.messages = MessageManager()
        self.stats_manager = StatsManager()

    def audio_status_callback(self, msg):
        self.audio_playing = msg.data  #True cuando habla, False cuando termina

    def result_callback(self, msg):
        self.result_received = msg.data

    def duration_callback(self, msg):
        self.duration_received = msg.data

    def show_message(self, msg):

        print(f"\n {msg}\n")

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
            
        #Pausa breve para naturalidad
        #time.sleep(0.5)



    def play(self):
        #name = input(" Nombre del niño: ")
        name = "pochi"
        self.show_message(random.choice(self.messages.welcome_messages))
        self.show_message("Memoriza la secuencia completa y haz las posturas en ese mismo orden")



        while self.game_active:
            self.show_message(f" Nivel {len(self.sequence) + 1}")

            if self.success or len(self.sequence) == 0:
                new_pose = random.choice(range(len(self.POSTURES)))
                full_sequence = self.sequence + [new_pose]

            # Determinar ayuda si aplica
            if self.total_fails == 1:
                timeout = self.TIME_LIMIT
                sequence_now = full_sequence
                self.show_message("ayuda 1, te repito la misma secuencia")

            elif self.total_fails == 2:
                timeout = self.TIME_LIMIT + 6
                sequence_now = full_sequence
                self.show_message("ayuda 2, te doy más tiempo")

            elif self.total_fails == 3:
                timeout = self.TIME_LIMIT
                sequence_now = full_sequence[:-1]
                self.show_message("ayuda 3, reduzcamos una postura de la secuencia")

            else:
                timeout = self.TIME_LIMIT
                sequence_now = full_sequence

            self.success = True

            # Mostrar la secuencia completa
            for pose_id in sequence_now:
                self.show_message(f" {self.POSTURES[pose_id].upper()}")

                
                
            self.show_message("Repite la secuencia")
            
# ---------------------------------------------------------------------------------------------------------------
            # Validar postura por postura
            for idx, expected in enumerate(sequence_now):
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
                    self.show_message(random.choice(self.messages.next_messages))

                else:
                    self.success = False
                    self.errors_by_pose[expected] += 1

                    if self.first_error_level is None:
                        self.first_error_level = len(full_sequence)

                    self.show_message(random.choice(self.messages.error_messages))

                    
                    break  # Se rompe la secuencia tras el primer error
# ---------------------------------------------------------------------------------------------------------------

            #Determinar que hacer despues del acierto o error
            if self.success:
                self.sequence = full_sequence
                self.score += 1
                self.show_message(random.choice(self.messages.success_messages))

            else:
                self.total_fails += 1
                if self.total_fails > self.MAX_ATTEMPTS:
                    self.end_game(name)
                    self.game_active = False
                else:
                    rclpy.spin_once(self, timeout_sec=1)

    def end_game(self, name):
        self.show_message(f" Fin del juego, {name}. Puntaje final: {self.score}")
        
        self.show_message(random.choice(self.messages.final_messages))

        total_time = time.time() - self.start_time
        avg_time = sum(self.response_times) / len(self.response_times) if self.response_times else 0
        std_dev = statistics.stdev(self.response_times) if len(self.response_times) > 1 else 0
        total_attempts = len(self.response_times) + sum(self.errors_by_pose)

        error_stats = {self.POSTURES[i]: self.errors_by_pose[i] for i in range(len(self.POSTURES))}

        self.stats_manager.save_stats(
            name=name,
            score=self.score,
            emotions=self.emotions,
            total_duration=total_time,
            avg_response_time=avg_time,
            response_times=self.response_times,
            response_deviation=std_dev,
            total_attempts=total_attempts,
            errors_per_posture=error_stats,
            first_error_level=self.first_error_level
        )


def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    node.play()
    node.destroy_node()
    rclpy.shutdown()

