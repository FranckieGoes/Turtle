# Fichier : server_for_robot.py (MIS À JOUR)

# ... (toutes les importations et initialisations restent les mêmes)
import pigpio, time, atexit, sys, threading, math
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
from database_manager import DatabaseManager

pi = pigpio.pi()
app = Flask(__name__)
db_manager = DatabaseManager()
motor_gauche_pins = { 'dir_pin': 27, 'step_pin': 12, 'en_pin': 22 }
motor_droit_pins = { 'dir_pin': 16, 'step_pin': 13, 'en_pin': 21 }
robot = RobotController(pi, motor_gauche_pins, motor_droit_pins)
atexit.register(robot.cleanup)

# --- Variables globales ---
# ... (variables existantes)
current_angle, current_speed, current_diameter = 180.0, 3.0, 150.0
robot_status, status_before_pause = "Prêt", "Prêt"
is_recording, raw_path_log = False, []

# NOUVELLES VARIABLES POUR LE SUIVI EN DIRECT
live_robot_position = {'x': 0, 'y': 0, 'angle': -90}
live_zone_name = None

# --- Fonctions de Géométrie et Traitement ---
# ... (toutes les fonctions _vectors_to_polygon, generate_mowing_path, etc. restent les mêmes)

# --- Logique de tonte et de suivi ---

def _update_live_position(vectors, speed_kmh):
    """
    Thread qui simule la position du robot pendant la tonte.
    """
    global live_robot_position, robot_status
    
    x, y, angle = 0, 0, -90.0
    live_robot_position = {'x': x, 'y': y, 'angle': angle}
    speed_cm_s = (speed_kmh * 100000) / 3600

    for vector in vectors:
        # Gère la pause
        if "pause" in robot_status.lower():
            while "pause" in robot_status.lower(): time.sleep(0.5)
        # Gère l'arrêt d'urgence
        if robot_status == "Prêt": break

        # Simuler la rotation
        if vector.get("relative_angle", 0) != 0:
            target_angle = angle + vector["relative_angle"]
            # Ici, on suppose que la rotation est assez rapide. Pour plus de réalisme, on pourrait simuler la durée.
            angle = target_angle
            live_robot_position = {'x': x, 'y': y, 'angle': angle}
            time.sleep(1) # Simule le temps de la rotation

        # Simuler le déplacement
        if vector.get("distance", 0) > 0:
            distance = vector["distance"]
            duration = distance / speed_cm_s if speed_cm_s > 0 else 0
            start_time = time.time()
            end_time = start_time + duration
            
            start_x, start_y = x, y
            angle_rad = math.radians(angle)

            while time.time() < end_time:
                if robot_status == "Prêt": break # Arrêt d'urgence
                progress = (time.time() - start_time) / duration
                x = start_x + distance * progress * math.cos(angle_rad)
                y = start_y + distance * progress * math.sin(angle_rad)
                live_robot_position = {'x': x, 'y': y, 'angle': angle}
                time.sleep(0.2) # Fréquence de mise à jour de la position

            # S'assurer d'être à la position finale exacte
            x = start_x + distance * math.cos(angle_rad)
            y = start_y + distance * math.sin(angle_rad)
            live_robot_position = {'x': x, 'y': y, 'angle': angle}

def run_autonomous_mowing(zone_name, speed_kmh):
    global robot_status, status_before_pause, live_zone_name
    
    status_before_pause = robot_status = f"Tonte de '{zone_name}'"
    live_zone_name = zone_name
    vectors = db_manager.get_zone_by_name(zone_name)
    if not vectors:
        robot_status = "Prêt"; live_zone_name = None; return

    # Démarrer le thread de mise à jour de la position
    position_thread = threading.Thread(target=_update_live_position, args=(vectors, speed_kmh))
    position_thread.start()

    # Exécuter les mouvements réels
    for vector in vectors:
        if "pause" in robot_status.lower():
            while "pause" in robot_status.lower(): time.sleep(1)
        if robot_status == "Prêt": break
        if vector.get("relative_angle", 0) != 0:
            robot.turn_on_spot_for_angle(speed_kmh, vector["relative_angle"])
        if vector.get("distance", 0) > 0:
            robot.move_for_distance(speed_kmh, vector["distance"])
    
    position_thread.join() # Attendre la fin du thread de position
    robot_status = "Prêt"
    live_zone_name = None

# --- Routes API ---
@app.route('/supervision')
def supervision_page():
    return render_template('supervision.html')

@app.route('/api/live_status', methods=['GET'])
def get_live_status():
    return jsonify({
        "status": robot_status,
        "zone_name": live_zone_name,
        "position": live_robot_position
    })

# ... (Toutes les autres routes comme /, /api/command, /zones, etc. restent inchangées)