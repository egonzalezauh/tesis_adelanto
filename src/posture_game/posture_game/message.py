import random

class MessageManager:
    def __init__(self):
        self.welcome_messages = [
            "Hola campeón ¿Listo para comenzar?",
            "Bienvenido al reto de las posturas",
            "Es hora de divertirse y moverse",
            "Hola, hoy jugaremos juntos a recordar posturas, ¿te animas?",
            "Qué alegría verte, vamos a entrenar tu memoria."
        ]

        self.success_messages = [
            "Muy bien hecha la secuencia",
            "Excelente memoria para repetir la secuencia",
            "Hiciste perfectas las posturas",
            "Tus posturas son increibles",
            "Genial, sigue recordando secuencias",
            "Tu concentración es increíble"
        ]

        self.error_messages = [
            "Intenta de nuevo. Tú puedes",
            "No te preocupes, lo harás mejor esta vez.",
            "Ánimo, repasemos la secuencia.",
            "Casi, vamos a intentarlo de nuevo.",
            "No pasa nada, sigamos practicando"
        ]

        self.motivation_messages = [
            "Vamos que vas súper bien",
            "Sigue así, lo estás logrando",
            "Eres increíble",
            "No te detengas",
            "Estás rompiendo tu récord",
            "Qué buena memoria tienes"
        ]

        self.final_messages = [
            "Buen trabajo. Inténtalo nuevamente cuando quieras.",
            "Juego terminado. Estoy orgulloso de ti.",
            "Lo hiciste excelente ¿Jugamos otra vez luego?",
            "Terminamos por hoy. Pero puedes seguir practicando.",
            "Gran esfuerzo Cada día mejoras más."
        ]

        self.next_messages = [
            "Vamos a la siguiente",
            "Perfecto",
            "Muy bien",
            "Increible",
            "Genial, sigue así"
        ]

    def get_random_message(self, message_list):
        return random.choice(message_list)
