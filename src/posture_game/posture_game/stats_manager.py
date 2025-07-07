import json
import os
from datetime import datetime

class StatsManager:
    def __init__(self, output_folder="results"):
        self.output_folder = output_folder
        os.makedirs(self.output_folder, exist_ok=True)

    def save_stats(self, name, score, emotions, total_duration,
                   avg_response_time, response_times,
                   response_deviation, total_attempts,
                   errors_per_posture, first_error_level):

        data = {
            "Nombre": name,
            "Fecha": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Puntaje": score,
            "Duracion total del juego": round(total_duration, 2),
            "Tiempo promedio de respuesta": round(avg_response_time, 2),
            "Desviacion estandar de respuesta": round(response_deviation, 2),
            "Total de intentos": total_attempts,
            "Nivel del primer error": first_error_level,
            "Errores por posturas": errors_per_posture,
            "Emociones": emotions,
            "Tiempo de respuesta": [round(t, 2) for t in response_times],
        }

        filename = f"{self.output_folder}/{name.replace(' ', '_')}_{data['Fecha'].replace(':', '-')}.json"
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

        print(f"✅ Results saved in: {filename}")
