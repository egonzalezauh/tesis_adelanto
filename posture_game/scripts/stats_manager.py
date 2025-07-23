import json
import os
from datetime import datetime
from collections import Counter

class StatsManager:
    def __init__(self, output_folder="results"):
        self.output_folder = output_folder
        os.makedirs(self.output_folder, exist_ok=True)


    def save_stats(self, name, score, emotions, total_duration,avg_response_time, response_times,response_deviation, total_attempts,errors_per_posture, first_error_level,
                   detailed_attempts):

        # Calcular porcentajes de emociones y ordenarlos de mayor a menor
        total_emotions = sum(emotions.values())
        emotion_percentages = {}

        if total_emotions > 0:
            raw_percentages = {
                emotion: round((count / total_emotions) * 100, 2)
                for emotion, count in emotions.items()
            }
            sorted_items = sorted(raw_percentages.items(), key=lambda x: x[1], reverse=True)
            emotion_percentages = {
                emotion: f"{percentage} %" for emotion, percentage in sorted_items
            }
        else:
            emotion_percentages = {
                emotion: "0.0 %" for emotion in emotions.keys()
            }

        emociones_por_nivel = map_emotions_by_level(detailed_attempts, score)



        data = {
            "Nombre": name,
            "Fecha": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "Puntaje": score,
            "Duracion total del juego (segundos)": round(total_duration, 2),
            "Tiempo promedio de respuesta (segundos)": round(avg_response_time, 2),
            "Desviacion estandar de respuesta (segundos)": round(response_deviation, 2),
            "Total de intentos(fallidos y exitosos)": total_attempts,
            "Nivel del primer error": first_error_level,
            "Errores por posturas": errors_per_posture,
            "Emociones": emotions,
            "Porcentaje de emociones (%)": emotion_percentages,
            "Tiempo de respuesta": [round(t, 2) for t in response_times],
            "Intentos detallados": detailed_attempts,
            "emociones_por_nivel": emociones_por_nivel,


        }

        filename = f"{self.output_folder}/{name.replace(' ', '_')}_{data['Fecha'].replace(':', '-')}.json"
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

        print(f"✅ Results saved in: {filename}")




def map_emotions_by_level(detailed_attempts, niveles_reales):
    emociones_por_nivel = {}
    idx = 0

    for nivel in range(1, niveles_reales + 1):
        secuencia = []

        for _ in range(nivel):
            if idx < len(detailed_attempts):
                secuencia.append(detailed_attempts[idx]["Emoción predominante"])
                idx += 1
            else:
                break

        if secuencia:
            emo_pred = Counter(secuencia).most_common(1)[0][0]
            emociones_por_nivel[f"Nivel {nivel}"] = emo_pred

    return emociones_por_nivel
