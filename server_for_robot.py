# Fichier : server_for_robot.py (CORRIGÉ)

# -*- coding: utf-8 -*-
import pigpio
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
from database_manager import DatabaseManager
import time
import atexit
import sys
import threading
import math

# --- Initialisations ---
pi = pigpio.pi()
app = Flask(__name__)
db_manager = DatabaseManager()
motor_gauche_pins = { 'dir_pin': 27, 'step_pin': 12, 'en_pin': 22 }
motor_droit_pins = { 'dir_pin': 16, 'step_pin': 13, 'en_pin': 21 }
robot = RobotController(pi, motor_gauche_pins, motor_droit_pins)
atexit.register(robot.cleanup)

# --- Variables globales ---
current_angle = 90.0
current_speed = 3.0
current_diameter = 150.0
robot_status = "Prêt"
status_before_pause = "Prêt"
is_recording = False
raw_path_log = []

# --- Fonctions de traitement du parcours ---
def process_raw_path_to_vectors(path_log):
    if not path_log or len(path_log) < 2: return []
    vectors = []
    for i in range(len(path_log) - 1):
        current_event, next_event = path_log[i], path_log[i+1]
        action = current_event.get("action")
        if action in ["forward", "backward"]:
            duration = next_event["time"] - current_event["time"]
            speed_kmh = current_event.get("speed", 0)
            distance_cm = ((speed_kmh * 100000) / 3600) * duration
            vectors.append({"distance": distance_cm, "relative_angle": 0})
        elif action in ["turn_left", "turn_right"]:
            angle = current_event.get("angle", 0)
            relative_angle = -angle if action == "turn_left" else angle
            vectors.append({"distance": 0, "relative_angle": relative_angle})
    return vectors

def merge_vectors(vectors):
    if not vectors: return []
    merged, accumulated_distance = [], 0
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

# --- Fonctions de Géométrie ---
def _vectors_to_polygon(vectors):
    polygon = [{'x': 0, 'y': 0}]
    x, y, angle = 0, 0, -90
    for v in vectors:
        angle += v.get("relative_angle", 0)
        distance = v.get("distance", 0)
        angle_rad = math.radians(angle)
        x += distance * math.cos(angle_rad)
        y += distance * math.sin(angle_rad)
        polygon.append({'x': x, 'y': y})
    return polygon

def _path_to_vectors(path_waypoints):
    if not path_waypoints: return []
    vectors = []
    current_angle = -90
    for i in range(len(path_waypoints) - 1):
        p1, p2 = path_waypoints[i], path_waypoints[i+1]
        dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']
        distance = math.sqrt(dx**2 + dy**2)
        if distance > 0.1:
            target_angle = math.degrees(math.atan2(dy, dx))
            relative_angle = target_angle - current_angle
            while relative_angle <= -180: relative_angle += 360
            while relative_angle > 180: relative_angle -= 360
            if abs(relative_angle) > 0.1:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
            vectors.append({"distance": distance, "relative_angle": 0})
            current_angle = target_angle
    return vectors

def generate_mowing_path(perimeter_vectors, mower_width_cm):
    if not perimeter_vectors: return []
    polygon = _vectors_to_polygon(perimeter_vectors)
    min_y, max_y = min(p['y'] for p in polygon), max(p['y'] for p in polygon)
    mowing_waypoints, y, direction = [], min_y + mower_width_cm / 2, 1
    while y < max_y:
        intersections = []
        for i in range(len(polygon)):
            p1, p2 = polygon[i], polygon[i-1]
            if min(p1['y'], p2['y']) < y <= max(p1['y'], p2['y']):
                x_intersect = (y - p1['y']) * (p2['x'] - p1['x']) / (p2['y'] - p1['y']) + p1['x']
                intersections.append(x_intersect)
        intersections.sort()
        if len(intersections) >= 2:
            if direction == 1:
                for i in range(0, len(intersections), 2):
                    mowing_waypoints.extend([{'x': intersections[i], 'y': y}, {'x': intersections[i+1], 'y': y}])
            else:
                for i in range(len(intersections) - 1, 0, -2):
                    mowing_waypoints.extend([{'x': intersections[i], 'y': y}, {'x': intersections[i-1], 'y': y}])
        y += mower_width_cm
        direction *= -1
    return _path_to_vectors(mowing_waypoints)


# --- Routes API ---
@app.route('/')
def home(): return render_template('robot_controller.html')

@app.route('/api/command', methods=['POST'])
def handle_command():
    global current_angle, current_speed, is_recording, current_diameter, robot_status, status_before_pause, raw_path_log 
    data = request.json
    command, params = data.get('command'), data.get('params', {})
    response_message, status = "", "success"

    if robot_status.lower().endswith("en pause") and command != 'resume':
        return jsonify({"status": "error", "message": f"Robot en pause. Appuyez sur Reprendre."}), 409

    if is_recording and command in ["forward", "backward", "turn_left", "turn_right"]:
        raw_path_log.append({"time": time.time(), "action": command, "speed": current_speed, "angle": current_angle})

    if command == 'forward': robot.move_forward(current_speed); robot_status = "Déplacement manuel"
    elif command == 'backward': robot.move_backward(current_speed); robot_status = "Déplacement manuel"
    elif command == 'turn_left': robot.select_type_rotate(current_angle, current_diameter, "left", current_speed); robot_status = "Déplacement manuel"
    elif command == 'turn_right': robot.select_type_rotate(current_angle, current_diameter, "right", current_speed); robot_status = "Déplacement manuel"
    elif command == 'stop': robot.stop(); robot_status = "Prêt"
    elif command == 'set_angle': current_angle = float(params.get('angle', current_angle))
    elif command == 'set_speed': current_speed = float(params.get('speed', current_speed)); robot.update_speed(current_speed)
    elif command == 'set_diameter': current_diameter = float(params.get('diameter', current_diameter))
    elif command == 'simulate_obstacle':
        if robot_status != "Prêt":
            robot.pause(); status_before_pause = robot_status; robot_status = "Obstacle - En Pause"
        else: status = "error"; response_message = "Le robot est déjà à l'arrêt."
    elif command == 'resume': robot.resume(); robot_status = status_before_pause
    elif command == 'start_recording':
        robot_status, is_recording = "Enregistrement", True
        raw_path_log = [{"time": time.time(), "action": "start"}]
    elif command == 'stop_recording':
        is_recording, robot_status = False, "Prêt"
        raw_path_log.append({"time": time.time(), "action": "end"})
        zone_name = params.get('zone_name')
        if not zone_name: return jsonify({"status": "error", "message": "Nom de zone requis."})
        final_vectors = merge_vectors(process_raw_path_to_vectors(raw_path_log))
        if db_manager.save_zone(zone_name, final_vectors): response_message = f"Zone '{zone_name}' enregistrée."
        else: status, response_message = "error", f"La zone '{zone_name}' existe déjà."
    else: status, response_message = "error", "Commande inconnue"
    
    return jsonify({"status": status, "message": response_message or command.capitalize()})

@app.route('/zones')
def list_zones(): return render_template('zones_list.html', zones=db_manager.get_all_zones())

@app.route('/view_zone')
def view_zone_page(): return render_template('mowing_zone.html')

@app.route('/api/zone/<string:zone_name>', methods=['GET'])
def get_zone_data(zone_name):
    zone_vectors = db_manager.get_zone_by_name(zone_name)
    if zone_vectors: return jsonify(zone_vectors)
    else: return jsonify({"error": "Zone non trouvée"}), 404

def run_autonomous_mowing(zone_name, speed_kmh):
    global robot_status, status_before_pause
    status_before_pause = robot_status = f"Tonte de '{zone_name}'"
    vectors = db_manager.get_zone_by_name(zone_name)
    if not vectors:
        robot_status = "Prêt"; return

    for vector in vectors:
        if "pause" in robot_status.lower():
            while "pause" in robot_status.lower(): time.sleep(1)
        if robot_status == "Prêt": break
        if vector.get("relative_angle", 0) != 0: robot.turn_on_spot_for_angle(speed_kmh, vector["relative_angle"])
        if vector.get("distance", 0) > 0: robot.move_for_distance(speed_kmh, vector["distance"])
        time.sleep(0.5)
    robot_status = "Prêt"

@app.route('/api/start_zone/<string:zone_name>', methods=['POST'])
def start_zone(zone_name):
    global robot_status
    if robot_status != "Prêt": return jsonify({"status": "error", "message": f"Robot occupé: {robot_status}"}), 409
    threading.Thread(target=run_autonomous_mowing, args=(zone_name, current_speed)).start()
    return jsonify({"status": "success", "message": f"Lancement de la tonte pour '{zone_name}'."})

@app.route('/api/status', methods=['GET'])
def get_status(): return jsonify({ "status": robot_status, "speed": current_speed, "angle": current_angle, "diameter": current_diameter })
    
@app.route('/api/zones', methods=['GET'])
def get_zones_list(): return jsonify(db_manager.get_all_zones())

@app.route('/api/live_path', methods=['GET'])
def get_live_path():
    if is_recording: return jsonify(process_raw_path_to_vectors(raw_path_log))
    #print("route('/api/live_path', valeur raw_path_log = ", raw_path_log, jsonify([]))
    return jsonify([])

@app.route('/api/generate_path/<string:zone_name>', methods=['POST'])
def generate_path(zone_name):
    mower_width = request.json.get('width')
    if not mower_width: return jsonify({"status": "error", "message": "Largeur de coupe non spécifiée."}), 400
    perimeter = db_manager.get_zone_by_name(zone_name)
    if not perimeter: return jsonify({"status": "error", "message": "Zone non trouvée."}), 404
    mowing_path_vectors = generate_mowing_path(perimeter, float(mower_width))
    if not mowing_path_vectors: return jsonify({"status": "error", "message": "Impossible de générer un parcours."}), 500
    new_zone_name = f"{zone_name}_parcours"
    if db_manager.save_zone(new_zone_name, mowing_path_vectors): return jsonify({"status": "success", "message": f"Parcours '{new_zone_name}' généré."})
    else: return jsonify({"status": "error", "message": f"Le parcours '{new_zone_name}' existe déjà."}), 409

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)