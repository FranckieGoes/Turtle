# -*- coding: utf-8 -*-
import pigpio, time, atexit, sys, math, serial, threading
import numpy as np
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
from database_manager import DatabaseManager

try:
    from shapely.geometry import Polygon, LineString
    SHAPELY_AVAILABLE = True
except ImportError:
    print("Shapely non disponible - utilisation de la méthode basique uniquement")
    SHAPELY_AVAILABLE = False

# --- Initialisation de Flask et de la DB ---
app = Flask(__name__)
db_manager = DatabaseManager()

# Définition des broches GPIO pour les moteurs
motor_gauche_pins = { 'dir_pin': 27, 'step_pin': 12, 'en_pin': 22 }
motor_droit_pins = { 'dir_pin': 16, 'step_pin': 13, 'en_pin': 21 }

# Initialisation USB (optionnelle)
try:
    ser = serial.Serial('/dev/ttyUSB0', 115200)
except Exception as e:
    print(f"Pas de liaison ttyUSB0 disponible: {e}")
    ser = None

# --- Initialisation de pigpio ---
try:
    pi = pigpio.pi()
    if not pi.connected:
        print("Erreur : le démon pigpiod n'est pas en cours d'exécution. Veuillez le démarrer avec 'sudo pigpiod'.")
        sys.exit(1)
except Exception as e:
    print(f"Erreur de connexion à pigpio: {e}")
    sys.exit(1)

try:
    robot = RobotController(pi, motor_gauche_pins, motor_droit_pins)
    atexit.register(robot.cleanup)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    robot = None

# --- Variables globales ---
current_angle = 90.0
current_speed = 3.0
current_diameter = 150.0
robot_status = "Prêt"
status_before_pause = "Prêt"

# Variables pour l'enregistrement du parcours
is_recording = False
raw_path_log = []

# Variables globales pour le suivi en temps réel
current_robot_position = {'x': 0, 'y': 0, 'angle': -90}
movement_start_time = None
movement_start_position = {'x': 0, 'y': 0}
is_robot_moving = False
current_movement_direction = None

# Variables pour le suivi en direct de la tonte
live_robot_position = {'x': 0, 'y': 0, 'angle': -90}
live_zone_name = None

# --- Fonctions de traitement du parcours CORRIGÉES ---
def process_raw_path_to_vectors(path_log):
    """
    CORRECTION: Traite les événements de façon plus robuste pour éviter les segments parasites
    """
    if not path_log or len(path_log) < 2: 
        return []
    
    vectors = []
    current_robot_angle = -90.0  # Angle initial du robot
    
    print(f"DEBUG: Processing {len(path_log)} events")
    
    i = 0
    while i < len(path_log) - 1:
        current_event = path_log[i]
        action = current_event.get("action")
        
        # Ignorer les événements de début et fin
        if action in ["start", "end"]:
            i += 1
            continue
            
        print(f"DEBUG: Event {i}: {action}")
        
        if action in ["forward", "backward"]:
            # Chercher le prochain événement STOP pour calculer la durée réelle
            stop_event = None
            j = i + 1
            
            while j < len(path_log):
                if path_log[j].get("action") == "stop":
                    stop_event = path_log[j]
                    break
                elif path_log[j].get("action") in ["forward", "backward", "turn_left", "turn_right"]:
                    # Si on trouve une autre action avant le STOP, utiliser cet événement
                    stop_event = path_log[j]
                    break
                j += 1
            
            if stop_event:
                # Calculer la distance basée sur la durée réelle de mouvement
                duration = stop_event["time"] - current_event["time"]
                speed_kmh = current_event.get("speed", 0)
                distance_cm = ((speed_kmh * 100000) / 3600) * duration
                
                # Filtrer les micro-mouvements (moins de 5cm)
                if distance_cm >= 2.0:
                    # Direction (avant = positive, arrière = négative pour calculs)
                    final_distance = distance_cm if action == "forward" else distance_cm
                    vectors.append({"distance": final_distance, "relative_angle": 0})
                    print(f"DEBUG: Added movement: {final_distance:.1f}cm ({action})")
                else:
                    print(f"DEBUG: Filtered micro-movement: {distance_cm:.1f}cm")
            
        elif action in ["turn_left", "turn_right"]:
            # Utiliser l'angle configuré comme angle relatif
            turn_angle = current_event.get("angle", 90)
            
            # Appliquer la direction de rotation
            if action == "turn_left":
                relative_angle = -turn_angle  # Rotation dans le sens anti-horaire
            else:
                relative_angle = turn_angle   # Rotation dans le sens horaire
            
            # Ajouter le vecteur de rotation seulement s'il est significatif
            if abs(relative_angle) >= 2.0:  # Filtrer les micro-rotations
                vectors.append({"distance": 0, "relative_angle": relative_angle})
                print(f"DEBUG: Added rotation: {relative_angle}° ({action})")
                
                # Mettre à jour l'angle du robot pour les calculs suivants
                current_robot_angle += relative_angle
            else:
                print(f"DEBUG: Filtered micro-rotation: {relative_angle}°")
        
        i += 1
    
    print(f"DEBUG: Generated {len(vectors)} vectors")
    return vectors

def merge_vectors(vectors):
    """Fusionne les vecteurs consécutifs de même direction avec filtrage amélioré"""
    if not vectors: 
        return []
    
    merged = []
    accumulated_distance = 0
    last_angle = None
    
    print(f"DEBUG: Merging {len(vectors)} vectors")
    
    for i, vector in enumerate(vectors):
        # Si c'est un mouvement en ligne droite
        if vector.get("relative_angle", 0) == 0 and vector.get("distance", 0) > 0:
            accumulated_distance += vector["distance"]
            print(f"DEBUG: Accumulating distance: +{vector['distance']:.1f}cm = {accumulated_distance:.1f}cm total")
        else:
            # Si on avait accumulé une distance significative, l'ajouter
            if accumulated_distance >= 10.0:  # Seuil minimum de 10cm
                merged.append({"distance": accumulated_distance, "relative_angle": 0})
                print(f"DEBUG: Added merged movement: {accumulated_distance:.1f}cm")
                accumulated_distance = 0
            elif accumulated_distance > 0:
                print(f"DEBUG: Discarded small movement: {accumulated_distance:.1f}cm")
                accumulated_distance = 0
            
            # Ajouter le vecteur de rotation s'il est significatif
            rotation = vector.get("relative_angle", 0)
            if abs(rotation) >= 5.0:  # Seuil minimum de 5°
                # Fusionner les rotations consécutives dans la même direction
                if (last_angle is not None and 
                    merged and 
                    merged[-1].get("distance", 0) == 0 and
                    ((rotation > 0 and merged[-1].get("relative_angle", 0) > 0) or
                     (rotation < 0 and merged[-1].get("relative_angle", 0) < 0))):
                    # Fusionner avec la rotation précédente
                    merged[-1]["relative_angle"] += rotation
                    print(f"DEBUG: Merged rotation: {merged[-1]['relative_angle']:.1f}°")
                else:
                    merged.append({"distance": 0, "relative_angle": rotation})
                    print(f"DEBUG: Added rotation: {rotation:.1f}°")
                
                last_angle = rotation
            elif abs(rotation) > 0:
                print(f"DEBUG: Discarded small rotation: {rotation:.1f}°")
    
    # Ajouter la distance accumulée restante
    if accumulated_distance >= 10.0:
        merged.append({"distance": accumulated_distance, "relative_angle": 0})
        print(f"DEBUG: Added final movement: {accumulated_distance:.1f}cm")
    elif accumulated_distance > 0:
        print(f"DEBUG: Discarded final small movement: {accumulated_distance:.1f}cm")
    
    print(f"DEBUG: Final merged vectors: {len(merged)}")
    
    # Post-traitement : supprimer les rotations nulles résiduelles
    final_vectors = []
    for vector in merged:
        if vector.get("distance", 0) > 0 or abs(vector.get("relative_angle", 0)) >= 5.0:
            final_vectors.append(vector)
    
    return final_vectors

# --- Fonctions de Géométrie ---
def _vectors_to_polygon(vectors):
    """Convertit les vecteurs en points de polygone"""
    polygon = [{'x': 0, 'y': 0}]
    x, y, angle = 0, 0, -90
    
    for v in vectors:
        angle += v.get("relative_angle", 0)
        distance = v.get("distance", 0)
        if distance > 0:
            angle_rad = math.radians(angle)
            x += distance * math.cos(angle_rad)
            y += distance * math.sin(angle_rad)
            polygon.append({'x': x, 'y': y})
    
    return polygon

# --- Fonctions de génération de parcours ---
def generate_mowing_path(perimeter_vectors, mower_width_cm):
    """Génère un parcours de tonte basique en lignes parallèles"""
    if not perimeter_vectors: 
        return []
    
    polygon = _vectors_to_polygon(perimeter_vectors)
    min_y = min(p['y'] for p in polygon)
    max_y = max(p['y'] for p in polygon)
    
    mowing_waypoints = []
    y = min_y + mower_width_cm / 2
    direction = 1
    
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
                    if i + 1 < len(intersections):
                        mowing_waypoints.extend([
                            {'x': intersections[i], 'y': y}, 
                            {'x': intersections[i+1], 'y': y}
                        ])
            else:
                for i in range(len(intersections) - 1, 0, -2):
                    if i - 1 >= 0:
                        mowing_waypoints.extend([
                            {'x': intersections[i], 'y': y}, 
                            {'x': intersections[i-1], 'y': y}
                        ])
        
        y += mower_width_cm
        direction *= -1
    
    return _path_to_vectors(mowing_waypoints)

def _path_to_vectors(path_waypoints):
    """Convertit une liste de waypoints en vecteurs de mouvement"""
    if not path_waypoints: 
        return []
    
    vectors = []
    current_angle = -90
    
    for i in range(len(path_waypoints) - 1):
        p1, p2 = path_waypoints[i], path_waypoints[i+1]
        dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 0.1:
            target_angle = math.degrees(math.atan2(dy, dx))
            relative_angle = target_angle - current_angle
            
            # Normaliser l'angle entre -180 et 180
            while relative_angle <= -180: 
                relative_angle += 360
            while relative_angle > 180: 
                relative_angle -= 360
            
            if abs(relative_angle) > 0.1:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
            
            vectors.append({"distance": distance, "relative_angle": 0})
            current_angle = target_angle
    
    return vectors

# --- Fonctions de suivi en temps réel ---
def _update_live_position(vectors, speed_kmh):
    """Thread qui simule la position du robot pendant la tonte autonome"""
    global live_robot_position, robot_status
    
    x, y, angle = 0, 0, -90.0
    live_robot_position = {'x': x, 'y': y, 'angle': angle}
    speed_cm_s = (speed_kmh * 100000) / 3600

    for vector in vectors:
        # Gère la pause
        if "pause" in robot_status.lower():
            while "pause" in robot_status.lower(): 
                time.sleep(0.5)
        
        # Gère l'arrêt d'urgence
        if robot_status == "Prêt": 
            break

        # Simuler la rotation
        if vector.get("relative_angle", 0) != 0:
            target_angle = angle + vector["relative_angle"]
            angle = target_angle
            live_robot_position = {'x': x, 'y': y, 'angle': angle}
            time.sleep(1)  # Simule le temps de la rotation

        # Simuler le déplacement
        if vector.get("distance", 0) > 0:
            distance = vector["distance"]
            duration = distance / speed_cm_s if speed_cm_s > 0 else 0
            start_time = time.time()
            end_time = start_time + duration
            
            start_x, start_y = x, y
            angle_rad = math.radians(angle)

            while time.time() < end_time:
                if robot_status == "Prêt": 
                    break
                
                progress = (time.time() - start_time) / duration
                x = start_x + distance * progress * math.cos(angle_rad)
                y = start_y + distance * progress * math.sin(angle_rad)
                live_robot_position = {'x': x, 'y': y, 'angle': angle}
                time.sleep(0.2)

            # Position finale exacte
            x = start_x + distance * math.cos(angle_rad)
            y = start_y + distance * math.sin(angle_rad)
            live_robot_position = {'x': x, 'y': y, 'angle': angle}

def run_autonomous_mowing(zone_name, speed_kmh):
    """Exécute la tonte autonome d'une zone"""
    global robot_status, status_before_pause, live_zone_name
    
    status_before_pause = robot_status = f"Tonte de '{zone_name}'"
    live_zone_name = zone_name
    vectors = db_manager.get_zone_by_name(zone_name)
    
    if not vectors:
        robot_status = "Prêt"
        live_zone_name = None
        return

    # Démarrer le thread de mise à jour de la position
    position_thread = threading.Thread(target=_update_live_position, args=(vectors, speed_kmh))
    position_thread.start()

    # Exécuter les mouvements réels
    for vector in vectors:
        if "pause" in robot_status.lower():
            while "pause" in robot_status.lower(): 
                time.sleep(1)
        if robot_status == "Prêt": 
            break
        
        if vector.get("relative_angle", 0) != 0: 
            robot.turn_on_spot_for_angle(speed_kmh, vector["relative_angle"])
        if vector.get("distance", 0) > 0: 
            robot.move_for_distance(speed_kmh, vector["distance"])
    
    # Attendre la fin du thread de position
    position_thread.join()
    robot_status = "Prêt"
    live_zone_name = None

# --- Routes API ---
@app.route('/')
def home(): 
    return render_template('robot_controller.html')

@app.route('/api/command', methods=['POST'])
def handle_command():
    global current_angle, current_speed, is_recording, current_diameter, robot_status, status_before_pause, raw_path_log
    global current_robot_position, movement_start_time, movement_start_position, is_robot_moving, current_movement_direction
    
    data = request.json
    command = data.get('command')
    params = data.get('params', {})
    response_message = ""
    status = "success"

    # Vérifier si le robot est en pause
    if robot_status.lower().endswith("en pause") and command != 'resume':
        return jsonify({"status": "error", "message": f"Robot en pause. Appuyez sur Reprendre."}), 409

    # Enregistrement des actions pendant l'enregistrement
    if is_recording and command in ["forward", "backward", "turn_left", "turn_right"]:
        raw_path_log.append({
            "time": time.time(), 
            "action": command, 
            "speed": current_speed, 
            "angle": current_angle
        })

    # Tracking des mouvements pour l'affichage temps réel
    if command in ['forward', 'backward']:
        is_robot_moving = True
        current_movement_direction = command
        movement_start_time = time.time()
        movement_start_position = current_robot_position.copy()
        
    elif command in ['turn_left', 'turn_right']:
        is_robot_moving = False
        # Mettre à jour l'angle instantanément
        angle_change = current_angle if command == 'turn_right' else -current_angle
        current_robot_position['angle'] += angle_change
        
    elif command == 'stop':
        if is_robot_moving and movement_start_time:
            # Finaliser la position quand on s'arrête
            elapsed_time = time.time() - movement_start_time
            distance_traveled = (current_speed * 100000 / 3600) * elapsed_time
            
            if current_movement_direction in ['forward', 'backward']:
                angle_rad = math.radians(current_robot_position['angle'])
                direction_multiplier = 1 if current_movement_direction == 'forward' else -1
                
                current_robot_position['x'] += distance_traveled * math.cos(angle_rad) * direction_multiplier
                current_robot_position['y'] += distance_traveled * math.sin(angle_rad) * direction_multiplier
        
        is_robot_moving = False
        movement_start_time = None

    # Traitement des commandes
    if command == 'forward': 
        robot.move_forward(current_speed)
        robot_status = "Déplacement manuel"
    elif command == 'backward': 
        robot.move_backward(current_speed)
        robot_status = "Déplacement manuel"
    elif command == 'turn_left': 
        robot.select_type_rotate(current_angle, current_diameter, "left", current_speed)
        robot_status = "Déplacement manuel"
    elif command == 'turn_right': 
        robot.select_type_rotate(current_angle, current_diameter, "right", current_speed)
        robot_status = "Déplacement manuel"
    elif command == 'stop': 
        robot.stop()
        robot_status = "Prêt"
    elif command == 'set_angle': 
        current_angle = float(params.get('angle', current_angle))
    elif command == 'set_speed': 
        current_speed = float(params.get('speed', current_speed))
        robot.update_speed(current_speed)
    elif command == 'set_diameter': 
        current_diameter = float(params.get('diameter', current_diameter))
    elif command == 'simulate_obstacle':
        if robot_status != "Prêt":
            robot.pause()
            status_before_pause = robot_status
            robot_status = "Obstacle - En Pause"
        else: 
            status = "error"
            response_message = "Le robot est déjà à l'arrêt."
    elif command == 'resume': 
        robot.resume()
        robot_status = status_before_pause
    elif command == 'start_recording':
        robot_status = "Enregistrement"
        is_recording = True
        raw_path_log = [{"time": time.time(), "action": "start"}]
        # Reset de la position pour l'enregistrement
        current_robot_position = {'x': 0, 'y': 0, 'angle': -90}
    elif command == 'stop_recording':
        is_recording = False
        robot_status = "Prêt"
        raw_path_log.append({"time": time.time(), "action": "end"})
        
        zone_name = params.get('zone_name')
        if not zone_name: 
            return jsonify({"status": "error", "message": "Nom de zone requis."})
        
        # CORRECTION: Utiliser la fonction corrigée de traitement
        final_vectors = merge_vectors(process_raw_path_to_vectors(raw_path_log))
        
        # Utiliser l'ancienne méthode pour l'instant
        if db_manager.save_zone(zone_name, final_vectors): 
            from datetime import datetime
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            total_distance = sum(v.get('distance', 0) for v in final_vectors) / 100
            response_message = f"Zone '{zone_name}' enregistrée le {current_time} ({total_distance:.1f}m)"
        else: 
            status = "error"
            response_message = f"La zone '{zone_name}' existe déjà."
    else: 
        status = "error"
        response_message = "Commande inconnue"
    
    return jsonify({"status": status, "message": response_message or command.capitalize()})

@app.route('/zones')
def list_zones(): 
    return render_template('zones_list.html', zones=db_manager.get_all_zones())

@app.route('/view_zone')
def view_zone_page(): 
    return render_template('mowing_zone.html')

@app.route('/supervision')
def supervision_page():
    return render_template('supervision.html')

@app.route('/api/status', methods=['GET'])
def get_status(): 
    return jsonify({
        "status": robot_status, 
        "speed": current_speed, 
        "angle": current_angle, 
        "diameter": current_diameter
    })

@app.route('/api/zones', methods=['GET'])
def get_zones_list(): 
    return jsonify(db_manager.get_all_zones())

@app.route('/api/zone/<string:zone_name>', methods=['GET'])
def get_zone_data(zone_name):
    zone_vectors = db_manager.get_zone_by_name(zone_name)
    if zone_vectors: 
        return jsonify(zone_vectors)
    else: 
        return jsonify({"error": "Zone non trouvée"}), 404

@app.route('/api/live_path', methods=['GET'])
def get_live_path():
    """Retourne le parcours en cours d'enregistrement"""
    if is_recording: 
        # CORRECTION: Utiliser la fonction corrigée
        return jsonify(process_raw_path_to_vectors(raw_path_log))
    return jsonify([])

@app.route('/api/live_position', methods=['GET'])
def get_live_position():
    """Retourne la position estimée en temps réel du robot"""
    global current_robot_position, movement_start_time, movement_start_position, is_robot_moving
    
    if not is_robot_moving or not movement_start_time:
        return jsonify({**current_robot_position, 'is_moving': False})
    
    # Calculer la position estimée basée sur le temps écoulé et la vitesse
    elapsed_time = time.time() - movement_start_time
    distance_traveled = (current_speed * 100000 / 3600) * elapsed_time  # cm
    
    # Mettre à jour la position estimée
    if current_movement_direction in ['forward', 'backward']:
        angle_rad = math.radians(current_robot_position['angle'])
        direction_multiplier = 1 if current_movement_direction == 'forward' else -1
        
        estimated_x = movement_start_position['x'] + (distance_traveled * math.cos(angle_rad) * direction_multiplier)
        estimated_y = movement_start_position['y'] + (distance_traveled * math.sin(angle_rad) * direction_multiplier)
        
        return jsonify({
            'x': estimated_x,
            'y': estimated_y,
            'angle': current_robot_position['angle'],
            'is_moving': True,
            'distance_from_start': distance_traveled
        })
    
    return jsonify({**current_robot_position, 'is_moving': True})

@app.route('/api/live_status', methods=['GET'])
def get_live_status():
    """Status pour la supervision de tonte"""
    return jsonify({
        "status": robot_status,
        "zone_name": live_zone_name,
        "position": live_robot_position
    })

@app.route('/api/start_zone/<string:zone_name>', methods=['POST'])
def start_zone(zone_name):
    """Démarre la tonte d'une zone"""
    global robot_status
    if robot_status != "Prêt": 
        return jsonify({"status": "error", "message": f"Robot occupé: {robot_status}"}), 409
    
    threading.Thread(target=run_autonomous_mowing, args=(zone_name, current_speed)).start()
    return jsonify({"status": "success", "message": f"Lancement de la tonte pour '{zone_name}'."})

@app.route('/api/debug_recording', methods=['GET'])
def debug_recording():
    """Route de debug pour examiner l'enregistrement en cours"""
    if not is_recording or not raw_path_log:
        return jsonify({"status": "error", "message": "Aucun enregistrement en cours"})
    
    # Analyser le log brut
    debug_info = {
        "total_events": len(raw_path_log),
        "events": []
    }
    
    for i, event in enumerate(raw_path_log):
        debug_info["events"].append({
            "index": i,
            "action": event.get("action"),
            "time": event.get("time", 0),
            "speed": event.get("speed"),
            "angle": event.get("angle")
        })
    
    # Analyser les vecteurs générés
    vectors = process_raw_path_to_vectors(raw_path_log)
    merged_vectors = merge_vectors(vectors)
    
    debug_info["vectors"] = {
        "raw_vectors": len(vectors),
        "merged_vectors": len(merged_vectors),
        "details": merged_vectors
    }
    
    return jsonify(debug_info)

@app.route('/api/clear_recording', methods=['POST'])
def clear_recording():
    """Efface l'enregistrement en cours côté serveur"""
    global raw_path_log, is_recording
    if is_recording:
        raw_path_log = [{"time": time.time(), "action": "start"}]
        return jsonify({"status": "success", "message": "Enregistrement effacé"})
    else:
        return jsonify({"status": "error", "message": "Aucun enregistrement en cours"})

@app.route('/api/generate_path/<string:zone_name>', methods=['POST'])
def generate_path(zone_name):
    """Génère un parcours de tonte basique"""
    mower_width = request.json.get('width')
    if not mower_width: 
        return jsonify({"status": "error", "message": "Largeur de coupe non spécifiée."}), 400
    
    perimeter = db_manager.get_zone_by_name(zone_name)
    if not perimeter: 
        return jsonify({"status": "error", "message": "Zone non trouvée."}), 404
    
    mowing_path_vectors = generate_mowing_path(perimeter, float(mower_width))
    if not mowing_path_vectors: 
        return jsonify({"status": "error", "message": "Impossible de générer un parcours."}), 500
    
    new_zone_name = f"{zone_name}_parcours"
    if db_manager.save_zone(new_zone_name, mowing_path_vectors): 
        return jsonify({"status": "success", "message": f"Parcours '{new_zone_name}' généré."})
    else: 
        return jsonify({"status": "error", "message": f"Le parcours '{new_zone_name}' existe déjà."}), 409

# --- Fonctions avancées pour la génération de parcours ---
def generate_advanced_mowing_path(perimeter_vectors, mower_width_cm, concentric_passes=3):
    """
    Génère un parcours de tonte optimisé avec validation de forme
    CORRECTION MAJEURE pour les passes concentriques
    """
    if not perimeter_vectors:
        return []
    
    print(f"DEBUG: Génération parcours avancé - {len(perimeter_vectors)} vecteurs")
    
    if not SHAPELY_AVAILABLE:
        print("Shapely indisponible - utilisation méthode basique")
        return generate_mowing_path(perimeter_vectors, mower_width_cm)
    
    # Convertir les vecteurs en points
    polygon_points = _vectors_to_polygon_points(perimeter_vectors)
    
    if len(polygon_points) < 3:
        print("ERREUR: Pas assez de points")
        return []
    
    # CORRECTION: S'assurer que le polygone est fermé
    if len(polygon_points) > 2 and polygon_points[0] != polygon_points[-1]:
        polygon_points.append(polygon_points[0])
        print("DEBUG: Polygone fermé automatiquement")
    
    try:
        # Créer le polygone avec validation robuste
        main_polygon = Polygon(polygon_points)
        if not main_polygon.is_valid:
            print("DEBUG: Correction du polygone invalide...")
            main_polygon = main_polygon.buffer(0)
            if not main_polygon.is_valid:
                return generate_mowing_path(perimeter_vectors, mower_width_cm)
        
        all_waypoints = []
        
        # Phase 1: Périmètre
        exterior_coords = list(main_polygon.exterior.coords[:-1])
        perimeter_waypoints = _generate_perimeter_path(exterior_coords, mower_width_cm * 0.6)
        all_waypoints.extend(perimeter_waypoints)
        
        # Phase 2: Passes concentriques CORRIGÉES
        current_polygon = main_polygon
        for pass_num in range(concentric_passes):
            buffer_distance = -mower_width_cm * (pass_num + 1) * 0.75
            inner_polygon = current_polygon.buffer(buffer_distance)
            
            if inner_polygon.is_empty or inner_polygon.area < (mower_width_cm ** 2):
                break
            
            # Gestion des multipolygons
            if hasattr(inner_polygon, 'geoms'):
                valid_polygons = [geom for geom in inner_polygon.geoms 
                                if hasattr(geom, 'area') and geom.area > (mower_width_cm ** 2)]
                if not valid_polygons:
                    break
                inner_polygon = max(valid_polygons, key=lambda p: p.area)
            
            if hasattr(inner_polygon, 'exterior') and inner_polygon.exterior is not None:
                inner_coords = list(inner_polygon.exterior.coords[:-1])
                if len(inner_coords) >= 3:
                    concentric_waypoints = _generate_perimeter_path(
                        inner_coords, mower_width_cm * 0.4, reverse=(pass_num % 2 == 1)
                    )
                    all_waypoints.extend(concentric_waypoints)
                    current_polygon = inner_polygon
                else:
                    break
            else:
                break
        
        # Phase 3: Centre en parallèle
        if not current_polygon.is_empty and current_polygon.area > (mower_width_cm * 1.5) ** 2:
            center_waypoints = _generate_parallel_path_for_polygon(current_polygon, mower_width_cm)
            all_waypoints.extend(center_waypoints)
        
        return _path_to_vectors(all_waypoints)
        
    except Exception as e:
        print(f"ERREUR génération avancée: {e}")
        return generate_mowing_path(perimeter_vectors, mower_width_cm)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)