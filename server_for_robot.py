# -*- coding: utf-8 -*-
import pigpio
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
from database_manager import DatabaseManager
import time
import atexit
import sys

# --- Initialisation de pigpio ---
try:
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("Le démon pigpiod n'est pas en cours d'exécution.")
except Exception as e:
    print(f"Erreur de connexion à pigpio: {e}. Veuillez le démarrer avec 'sudo pigpiod'.")
    sys.exit(1)

# --- Initialisation de Flask et de la DB ---
app = Flask(__name__)
db_manager = DatabaseManager() # base de donnée

# Définition des broches GPIO
motor_gauche_pins = { 'dir_pin': 27, 'step_pin': 12, 'en_pin': 22 }
motor_droit_pins = { 'dir_pin': 16, 'step_pin': 13, 'en_pin': 21 }

# --- Initialisation du contrôleur de robot ---
try:
    robot = RobotController(pi, motor_gauche_pins, motor_droit_pins)
    atexit.register(robot.cleanup)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    robot = None

# Variables globales pour l'état du robot
current_angle = 180.0
current_speed = 3.0
current_diameter = 150.0 # NOUVELLE VARIABLE GLOBALE

# Variables pour l'enregistrement du parcours
raw_path_log = []
is_recording = False

# --- Fonctions de traitement du parcours ---
def process_raw_path_to_vectors(path_log):
    if not path_log or len(path_log) < 2:
        return []

    vectors = []
    for i in range(len(path_log) - 1):
        current_event = path_log[i]
        next_event = path_log[i+1]
        action = current_event.get("action")

        if action in ["forward", "backward"]:
            duration = next_event["time"] - current_event["time"]
            speed_kmh = current_event.get("speed", 0)
            speed_cm_s = (speed_kmh * 100000) / 3600
            distance_cm = speed_cm_s * duration
            vectors.append({"distance": distance_cm, "relative_angle": 0})

        elif action in ["turn_left", "turn_right"]:
            angle = current_event.get("angle", 0)
            relative_angle = -angle if action == "turn_left" else angle
            vectors.append({"distance": 0, "relative_angle": relative_angle})
            
    return vectors

def merge_vectors(vectors):
    if not vectors: return []
    merged = []
    accumulated_distance = 0
    for vector in vectors:
        if vector["relative_angle"] == 0 and vector["distance"] > 0:
            accumulated_distance += vector["distance"]
        else:
            if accumulated_distance > 0:
                merged.append({"distance": accumulated_distance, "relative_angle": 0})
                accumulated_distance = 0
            if vector["relative_angle"] != 0 or vector["distance"] != 0:
                merged.append(vector)
    if accumulated_distance > 0:
        merged.append({"distance": accumulated_distance, "relative_angle": 0})
    return merged

# --- Routes de l'API Flask ---
@app.route('/')
def home():
    return render_template('robot_controller.html')

@app.route('/api/command', methods=['POST'])
def handle_command():
    global current_angle, current_speed, is_recording, current_diameter

    data = request.json
    command = data.get('command')
    params = data.get('params', {})
    response_message = ""
    status = "success"

    print(f"Commande reçue: {command} avec les paramètres: {params}")

    if not robot:
        return jsonify({"status": "error", "message": "Le contrôleur de robot n'est pas initialisé."}), 500

    # Log l'action si l'enregistrement est actif
    if is_recording and command in ["forward", "backward", "turn_left", "turn_right", "stop"]:
        log_entry = {"time": time.time(), "action": command}
        if command in ["forward", "backward"]: log_entry["speed"] = current_speed
        if command in ["turn_left", "turn_right"]: log_entry["angle"] = current_angle
        raw_path_log.append(log_entry)

    if command == 'forward':
        robot.move_forward(current_speed)
        response_message = f"Avancer à {current_speed} km/h"
    elif command == 'backward':
        robot.move_backward(current_speed)
        response_message = f"Reculer à {current_speed} km/h"
    elif command == 'turn_left':
        # MODIFICATION : Utilisation de current_diameter
        robot.select_type_rotate(angle_IHM=current_angle, diametre_IHM=current_diameter, direction_IHM="left", vitesse_IHM=current_speed)
        response_message = f"Tourner à gauche de {current_angle}°"
    elif command == 'turn_right':
        # MODIFICATION : Utilisation de current_diameter
        robot.select_type_rotate(angle_IHM=current_angle, diametre_IHM=current_diameter, direction_IHM="right", vitesse_IHM=current_speed)
        response_message = f"Tourner à droite de {current_angle}°"
    elif command == 'stop':
        robot.stop()
        response_message = "Arrêt du robot"
    elif command == 'set_angle':
        current_angle = float(params.get('angle', current_angle))
        response_message = f"Angle réglé sur {current_angle}°"
    elif command == 'set_speed':
        current_speed = float(params.get('speed', current_speed))
        robot.update_speed(current_speed)
        response_message = f"Vitesse réglée sur {current_speed} km/h"
    # NOUVELLE COMMANDE
    elif command == 'set_diameter':
        current_diameter = float(params.get('diameter', current_diameter))
        response_message = f"Diamètre réglé sur {current_diameter} cm"
    elif command == 'start_recording':
        is_recording = True
        raw_path_log.clear()
        raw_path_log.append({"action": "start", "time": time.time()})
        response_message = "Enregistrement démarré"
    elif command == 'stop_recording':
        is_recording = False
        raw_path_log.append({"action": "end", "time": time.time()})
        zone_name = params.get('zone_name')
        if not zone_name:
            return jsonify({"status": "error", "message": "Le nom de la zone est requis."})
        
        basic_vectors = process_raw_path_to_vectors(raw_path_log)
        final_vectors = merge_vectors(basic_vectors)
        
        if db_manager.save_zone(zone_name, final_vectors):
            response_message = f"Zone '{zone_name}' enregistrée avec succès."
        else:
            status = "error"
            response_message = f"Erreur: La zone '{zone_name}' existe déjà."
    else:
        status = "error"
        response_message = "Commande inconnue"

    return jsonify({"status": status, "message": response_message})

@app.route('/zones')
def list_zones():
    all_zones = db_manager.get_all_zones()
    return render_template('zones_list.html', zones=all_zones)

@app.route('/view_zone')
def view_zone_page():
    return render_template('mowing_zone.html')

@app.route('/api/zone/<string:zone_name>', methods=['GET'])
def get_zone_data(zone_name):
    zone_vectors = db_manager.get_zone_by_name(zone_name)
    if zone_vectors:
        return jsonify(zone_vectors)
    else:
        return jsonify({"error": "Zone non trouvée"}), 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)