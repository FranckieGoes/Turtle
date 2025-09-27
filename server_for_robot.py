# -*- coding: utf-8 -*-
import pigpio, time, atexit, sys, math, serial, threading
import numpy as np
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
from database_manager import DatabaseManager

# Ajoutez ces lignes tout au début, après les imports système
print("=== TEST SHAPELY ===")
import sys
print(f"Python: {sys.executable}")

try:
    from shapely.geometry import Polygon, LineString
    print("✓ Shapely importé avec succès")
    
    # Test simple
    test_poly = Polygon([(0, 0), (10, 0), (10, 10), (0, 10)])
    print(f"✓ Test polygone: aire = {test_poly.area}")
    
    SHAPELY_AVAILABLE = True
except ImportError as e:
    print(f"✗ Import Shapely échoué: {e}")
    SHAPELY_AVAILABLE = False
except Exception as e:
    print(f"✗ Erreur Shapely: {e}")
    SHAPELY_AVAILABLE = False

print(f"SHAPELY_AVAILABLE = {SHAPELY_AVAILABLE}")
print("====================")

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
        print("Erreur : le démon pigpiod n'est pas en cours d'exécution. Veuillez le démarrer avec sudo : sudo pigpiod")
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
    """
    CORRECTION MAJEURE: Ne pas fusionner les rotations avec les mouvements
    Cela cause des erreurs de calcul de position
    """
    if not vectors:
        return []
    
    print("=== FUSION VECTEURS (CORRIGÉE) ===")
    print(f"Vecteurs avant fusion: {len(vectors)}")
    
    merged = []
    
    for i, vector in enumerate(vectors):
        print(f"Vecteur {i}: {vector}")
        
        # CORRECTION: Garder les rotations pures séparées
        if vector.get("distance", 0) == 0 and vector.get("relative_angle", 0) != 0:
            # C'est une rotation pure - la garder séparée
            merged.append({
                "distance": 0,
                "relative_angle": vector["relative_angle"]
            })
            print(f"  → Rotation pure conservée: {vector['relative_angle']}°")
            
        elif vector.get("distance", 0) > 0:
            # C'est un mouvement - peut avoir une rotation associée
            merged.append({
                "distance": vector["distance"],
                "relative_angle": vector.get("relative_angle", 0)
            })
            print(f"  → Mouvement conservé: {vector['distance']}cm, rotation: {vector.get('relative_angle', 0)}°")
    
    print(f"Vecteurs après fusion: {len(merged)}")
    print("=================================")
    
    return merged

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

def _vectors_to_polygon_points(vectors):
    """
    CORRECTION: Convertit les vecteurs en points de polygone de façon plus robuste
    """
    points = [(0, 0)]  # Point de départ
    x, y, angle = 0, 0, -90
    
    for vector in vectors:
        # Appliquer la rotation
        if vector.get("relative_angle", 0) != 0:
            angle += vector["relative_angle"]
        
        # Appliquer le mouvement
        distance = vector.get("distance", 0)
        if distance > 0:
            angle_rad = math.radians(angle)
            x += distance * math.cos(angle_rad)
            y += distance * math.sin(angle_rad)
            points.append((x, y))
    
    print(f"_vectors_to_polygon_points: {len(vectors)} vecteurs → {len(points)} points")
    return points

def calculateRealPositionFromVectors(vectors):
    """
    CORRECTION: Le bug était dans l'ordre des opérations lors des rotations
    """
    if not vectors:
        return {'x': 0, 'y': 0, 'angle': -90}
    
    x, y, angle = 0, 0, -90.0
    
    print(f"=== CALCUL POSITION (CORRIGÉ) ===")
    print(f"Vecteurs à traiter: {len(vectors)}")
    
    for i, vector in enumerate(vectors):
        print(f"Vecteur {i}: {vector}")
        
        # CORRECTION 1: Traiter rotation ET mouvement dans le même vecteur
        relative_angle = vector.get("relative_angle", 0)
        distance = vector.get("distance", 0)
        
        # CORRECTION 2: Si c'est une rotation pure (distance = 0)
        if distance == 0 and abs(relative_angle) > 0.1:
            angle += relative_angle
            # Normaliser l'angle
            while angle > 180: angle -= 360
            while angle <= -180: angle += 360
            print(f"  ROTATION PURE: {relative_angle}° → angle total: {angle:.1f}°")
            # PAS DE MOUVEMENT - continuer au vecteur suivant
            continue
        
        # CORRECTION 3: Si c'est un mouvement (avec ou sans rotation)
        if distance > 0.1:
            # Appliquer d'abord la rotation si elle existe
            if abs(relative_angle) > 0.1:
                angle += relative_angle
                while angle > 180: angle -= 360
                while angle <= -180: angle += 360
                print(f"  Rotation avant mouvement: {relative_angle}° → angle: {angle:.1f}°")
            
            # Puis appliquer le mouvement avec le nouvel angle
            angle_rad = math.radians(angle)
            delta_x = distance * math.cos(angle_rad)
            delta_y = distance * math.sin(angle_rad)
            
            old_x, old_y = x, y
            x += delta_x
            y += delta_y
            
            print(f"  Mouvement: {distance:.1f}cm à {angle:.1f}° → ({old_x:.1f}, {old_y:.1f}) → ({x:.1f}, {y:.1f})")
    
    # Normalisation finale
    while angle > 180: angle -= 360
    while angle <= -180: angle += 360
    
    final_position = {'x': x, 'y': y, 'angle': angle}
    distance_to_start = math.sqrt(x*x + y*y)
    print(f"Position finale: {final_position}")
    print(f"Distance au départ: {distance_to_start:.1f}cm")
    print("===================================")
    
    return final_position  

def _generate_perimeter_path(coords, offset_distance, reverse=False):
    """
    Génère un chemin le long du périmètre avec un décalage vers l'intérieur
    
    Args:
        coords (list): Liste des coordonnées du périmètre
        offset_distance (float): Distance de décalage vers l'intérieur
        reverse (bool): Inverser le sens de parcours
        
    Returns:
        list: Liste de waypoints pour le parcours périmétrique
    """
    if len(coords) < 3:
        return []
    
    waypoints = []
    coords_list = list(coords)
    
    if reverse:
        coords_list.reverse()
    
    # Pour simplifier, on utilise directement les coordonnées
    # Dans un cas réel, on calculerait un offset vers l'intérieur
    for coord in coords_list:
        if hasattr(coord, '__iter__') and len(coord) >= 2:
            waypoints.append({'x': coord[0], 'y': coord[1]})
        else:
            print(f"Coordonnée invalide ignorée: {coord}")
    
    return waypoints

def _generate_parallel_path_for_polygon(polygon, mower_width):
    """
    Génère un parcours en lignes parallèles pour un polygone donné
    
    Args:
        polygon: Polygone Shapely
        mower_width (float): Largeur de coupe en cm
        
    Returns:
        list: Liste de waypoints pour le parcours parallèle
    """
    if not SHAPELY_AVAILABLE or polygon.is_empty:
        return []
    
    try:
        # Récupérer les limites du polygone
        minx, miny, maxx, maxy = polygon.bounds
        
        waypoints = []
        y = miny + mower_width / 2
        direction = 1  # 1 pour droite à gauche, -1 pour gauche à droite
        
        while y < maxy:
            # Créer une ligne horizontale à la hauteur y
            line = LineString([(minx - 100, y), (maxx + 100, y)])
            
            # Trouver les intersections avec le polygone
            intersection = polygon.intersection(line)
            
            if hasattr(intersection, 'geoms'):
                # Intersection multiple
                segments = list(intersection.geoms)
            elif hasattr(intersection, 'coords'):
                # Intersection simple
                segments = [intersection]
            else:
                segments = []
            
            # Traiter chaque segment
            for segment in segments:
                if hasattr(segment, 'coords') and len(list(segment.coords)) >= 2:
                    coords = list(segment.coords)
                    if direction == 1:
                        # Gauche à droite
                        waypoints.extend([
                            {'x': coords[0][0], 'y': coords[0][1]},
                            {'x': coords[-1][0], 'y': coords[-1][1]}
                        ])
                    else:
                        # Droite à gauche
                        waypoints.extend([
                            {'x': coords[-1][0], 'y': coords[-1][1]},
                            {'x': coords[0][0], 'y': coords[0][1]}
                        ])
            
            y += mower_width * 0.8  # 80% de recouvrement
            direction *= -1  # Alterner la direction
        
        return waypoints
        
    except Exception as e:
        print(f"Erreur génération parallèle: {e}")
        return []

# --- Fonctions de génération de parcours ---
def generate_mowing_path(perimeter_vectors, mower_width_cm):
    """
    CORRECTION: Génère un parcours de tonte basique en lignes parallèles
    Cette fonction ne doit être utilisée que pour la tonte intérieure, pas le périmètre
    """
    if not perimeter_vectors: 
        return []
    
    print(f"Génération parcours basique: largeur={mower_width_cm}cm")
    
    # Convertir les vecteurs en polygon points
    polygon_points = _vectors_to_polygon_points(perimeter_vectors)
    
    if len(polygon_points) < 3:
        print("Pas assez de points pour générer un parcours")
        return []
    
    # Calculer les limites
    x_coords = [p[0] for p in polygon_points]
    y_coords = [p[1] for p in polygon_points]
    
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    
    print(f"Zone: x[{min_x:.1f}, {max_x:.1f}], y[{min_y:.1f}, {max_y:.1f}]")
    
    # Générer les lignes parallèles
    mowing_waypoints = []
    y = min_y + mower_width_cm
    direction = 1
    
    while y < max_y - mower_width_cm:
        # Points de la ligne horizontale
        if direction == 1:
            mowing_waypoints.extend([
                {'x': min_x + mower_width_cm, 'y': y},
                {'x': max_x - mower_width_cm, 'y': y}
            ])
        else:
            mowing_waypoints.extend([
                {'x': max_x - mower_width_cm, 'y': y},
                {'x': min_x + mower_width_cm, 'y': y}
            ])
        
        y += mower_width_cm * 0.8  # 80% de recouvrement
        direction *= -1
    
    print(f"Parcours basique: {len(mowing_waypoints)} waypoints générés")
    
    return _path_to_vectors(mowing_waypoints)

def _path_to_vectors(path_waypoints):
    """
    CORRECTION: Convertit waypoints en vecteurs de façon plus robuste
    """
    if not path_waypoints or len(path_waypoints) < 2: 
        return []
    
    vectors = []
    current_angle = -90
    
    for i in range(len(path_waypoints) - 1):
        p1, p2 = path_waypoints[i], path_waypoints[i+1]
        dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 1.0:  # Ignorer les micro-mouvements
            target_angle = math.degrees(math.atan2(dy, dx))
            relative_angle = target_angle - current_angle
            
            # Normaliser l'angle
            while relative_angle > 180: 
                relative_angle -= 360
            while relative_angle <= -180: 
                relative_angle += 360
            
            # Ajouter rotation si significative
            if abs(relative_angle) > 2:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
            
            # Ajouter mouvement
            vectors.append({"distance": distance, "relative_angle": 0})
            current_angle = target_angle
    
    print(f"_path_to_vectors: {len(path_waypoints)} waypoints → {len(vectors)} vecteurs")
    return vectors

# --- Fonctions de suivi en temps réel ---
def _update_live_position(vectors, speed_kmh, simulation_active):
    """
    CORRECTION: Thread qui simule la position du robot avec contrôle externe
    Le thread ne s'arrête que quand simulation_active est désactivé
    """
    global live_robot_position, robot_status
    
    if not vectors:
        print("SUPERVISION: Aucun vecteur à traiter")
        return
    
    x, y, angle = 0, 0, -90.0
    live_robot_position = {'x': x, 'y': y, 'angle': angle}
    speed_cm_s = (speed_kmh * 100000) / 3600

    print(f"SUPERVISION: Début simulation avec {len(vectors)} vecteurs à {speed_kmh}km/h")

    for i, vector in enumerate(vectors):
        # CORRECTION 1: Vérifier le flag de contrôle au lieu du statut
        if not simulation_active.is_set():
            print(f"SUPERVISION: Arrêt demandé par le flag au vecteur {i}")
            break
        
        # Gérer la pause (vérifier le statut pour ça)
        while "pause" in robot_status.lower() and simulation_active.is_set(): 
            print(f"SUPERVISION: Pause détectée au vecteur {i}")
            time.sleep(0.5)
        
        if not simulation_active.is_set():
            break
            
        print(f"SUPERVISION: Vecteur {i+1}/{len(vectors)} - distance:{vector.get('distance', 0)}cm, angle:{vector.get('relative_angle', 0)}°")

        # CORRECTION 2: Simuler la rotation avec vérification continue
        if vector.get("relative_angle", 0) != 0:
            rotation = vector["relative_angle"]
            print(f"SUPERVISION: Rotation de {rotation}°")
            
            steps = max(1, int(abs(rotation) / 15))  # 15° par étape
            for step in range(steps):
                if not simulation_active.is_set():
                    break
                angle += rotation / steps
                live_robot_position = {'x': x, 'y': y, 'angle': angle}
                time.sleep(0.2)  # Temps de rotation plus court
            
            # Position finale exacte
            if simulation_active.is_set():
                angle += rotation - (rotation / steps * steps)
                live_robot_position = {'x': x, 'y': y, 'angle': angle}

        # CORRECTION 3: Simuler le déplacement avec vérification continue
        distance = vector.get("distance", 0)
        if distance > 0 and simulation_active.is_set():
            print(f"SUPERVISION: Déplacement de {distance}cm")
            
            duration = max(0.5, distance / speed_cm_s if speed_cm_s > 0 else 1)
            start_time = time.time()
            end_time = start_time + duration
            
            start_x, start_y = x, y
            angle_rad = math.radians(angle)

            # Simulation progressive du mouvement
            while time.time() < end_time and simulation_active.is_set():
                # Gérer la pause pendant le mouvement
                while "pause" in robot_status.lower() and simulation_active.is_set(): 
                    time.sleep(0.5)
                
                if not simulation_active.is_set():
                    break
                
                progress = (time.time() - start_time) / duration
                progress = min(1.0, progress)
                
                x = start_x + distance * progress * math.cos(angle_rad)
                y = start_y + distance * progress * math.sin(angle_rad)
                live_robot_position = {'x': x, 'y': y, 'angle': angle}
                time.sleep(0.1)  # Mise à jour plus fréquente

            # Position finale exacte si pas d'arrêt
            if simulation_active.is_set():
                x = start_x + distance * math.cos(angle_rad)
                y = start_y + distance * math.sin(angle_rad)
                live_robot_position = {'x': x, 'y': y, 'angle': angle}
                
                print(f"SUPERVISION: Position après déplacement: x={x:.1f}, y={y:.1f}")

    if simulation_active.is_set():
        print(f"SUPERVISION: Simulation terminée normalement - Position finale: x={x:.1f}, y={y:.1f}, angle={angle:.1f}°")
    else:
        print(f"SUPERVISION: Simulation interrompue - Position: x={x:.1f}, y={y:.1f}, angle={angle:.1f}°")

def run_autonomous_mowing(zone_name, speed_kmh):
    """
    CORRECTION: Exécute la tonte autonome d'une zone
    Maintenir le statut pendant toute la durée de la tonte
    """
    global robot_status, status_before_pause, live_zone_name
    
    print(f"TONTE: Début tonte autonome de '{zone_name}' à {speed_kmh}km/h")
    
    # CORRECTION 1: Maintenir le statut pendant toute la tonte
    original_status = robot_status
    robot_status = f"Tonte de '{zone_name}' en cours"
    status_before_pause = robot_status
    live_zone_name = zone_name
    
    vectors = db_manager.get_zone_by_name(zone_name)
    
    if not vectors:
        print(f"ERREUR: Zone '{zone_name}' introuvable")
        robot_status = "Prêt"
        live_zone_name = None
        return

    print(f"TONTE: {len(vectors)} vecteurs chargés pour la zone")

    # CORRECTION 2: Démarrer le thread de simulation AVEC un flag de contrôle
    simulation_active = threading.Event()
    simulation_active.set()
    
    position_thread = threading.Thread(
        target=_update_live_position, 
        args=(vectors, speed_kmh, simulation_active)
    )
    position_thread.daemon = True
    position_thread.start()

    # CORRECTION 3: Exécuter les mouvements réels SANS changer le statut
    try:
        print(f"TONTE: Début exécution de {len(vectors)} vecteurs")
        
        for i, vector in enumerate(vectors):
            # Vérifier les conditions d'arrêt (mais pas le statut "Prêt")
            while "pause" in robot_status.lower():
                print(f"TONTE: Pause au vecteur {i}")
                time.sleep(1)
                
            # CORRECTION: Ne pas arrêter sur "Prêt" pendant la tonte
            if "arrêt demandé" in robot_status.lower():
                print(f"TONTE: Arrêt explicite demandé au vecteur {i}")
                break
            
            print(f"TONTE: Exécution vecteur {i+1}/{len(vectors)}")
            
            # Exécuter rotation
            if vector.get("relative_angle", 0) != 0:
                angle = vector["relative_angle"]
                print(f"TONTE: Rotation réelle de {angle}°")
                robot.turn_on_spot_for_angle(speed_kmh, angle)
            
            # Exécuter déplacement
            if vector.get("distance", 0) > 0:
                distance = vector["distance"]
                print(f"TONTE: Déplacement réel de {distance}cm")
                robot.move_for_distance(speed_kmh, distance)
            
            # CORRECTION: Mettre à jour le statut avec progression
            progress_pct = ((i + 1) / len(vectors)) * 100
            robot_status = f"Tonte de '{zone_name}' - {progress_pct:.0f}% ({i+1}/{len(vectors)})"
        
        print("TONTE: Tous les vecteurs exécutés avec succès")
        
    except Exception as e:
        print(f"ERREUR TONTE: {e}")
        robot_status = f"Erreur tonte: {str(e)}"
    finally:
        # CORRECTION 4: Arrêter proprement la simulation
        simulation_active.clear()
        
        if position_thread.is_alive():
            print("TONTE: Attente fin simulation...")
            position_thread.join(timeout=10.0)  # Plus de temps d'attente
        
        # Remettre le statut final
        robot_status = "Prêt"
        live_zone_name = None
        print("TONTE: Terminée et statut remis à Prêt")

def normalize_angle(angle):
    """
    Normalise un angle entre -180 et 180 degrés
    CORRECTION: Cette fonction doit être utilisée partout
    """
    while angle > 180:
        angle -= 360
    while angle <= -180:
        angle += 360
    return angle

def calculateRealPositionFromVectors(vectors):
    """
    CORRECTION: Calcul avec normalisation stricte des angles
    """
    if not vectors:
        return {'x': 0, 'y': 0, 'angle': -90}
    
    x, y, angle = 0, 0, -90.0
    
    print(f"=== CALCUL POSITION (CORRIGÉ) ===")
    
    for i, vector in enumerate(vectors):
        # CORRECTION 1: Normaliser la rotation relative
        relative_angle = vector.get("relative_angle", 0)
        if abs(relative_angle) > 0.1:
            angle += relative_angle
            angle = normalize_angle(angle)  # NORMALISATION AJOUTÉE
            print(f"Vecteur {i}: rotation {relative_angle}° → angle normalisé: {angle:.1f}°")
        
        # CORRECTION 2: Mouvement avec angle normalisé
        distance = vector.get("distance", 0)
        if distance > 0.1:
            angle_rad = math.radians(angle)
            delta_x = distance * math.cos(angle_rad)
            delta_y = distance * math.sin(angle_rad)
            
            x += delta_x
            y += delta_y
            
            print(f"Vecteur {i}: mouvement {distance:.1f}cm → position ({x:.1f}, {y:.1f})")
    
    # CORRECTION 3: Normalisation finale
    angle = normalize_angle(angle)
    
    final_position = {'x': x, 'y': y, 'angle': angle}
    print(f"Position finale normalisée: {final_position}")
    print("===================================")
    
    return final_position

# --- Routes API ---

    # FONCTION DE TEST - À ajouter temporairement
@app.route('/api/test_position_calculation', methods=['POST'])
def test_position_calculation():
    """
    Test de validation du calcul de position
    """
    # Vecteurs de test basés sur vos logs
    test_vectors = [
        {"distance": 453.09, "relative_angle": 0},     # Mouvement vers le bas
        {"distance": 0, "relative_angle": 90},         # Rotation droite
        {"distance": 559.64, "relative_angle": 0},     # Mouvement vers la droite
        {"distance": 0, "relative_angle": 90},         # Rotation droite
        {"distance": 320.32, "relative_angle": 0},     # Mouvement vers le haut
        {"distance": 0, "relative_angle": 90},         # Rotation droite
        {"distance": 423.47, "relative_angle": 0},     # Mouvement vers la gauche
        {"distance": 0, "relative_angle": -90}         # Rotation gauche
    ]
    
    print("=== TEST CALCUL POSITION ===")
    position = calculateRealPositionFromVectors(test_vectors)
    distance = math.sqrt(position['x']**2 + position['y']**2)
    
    return jsonify({
        "test_vectors": test_vectors,
        "calculated_position": position,
        "distance_to_start": distance,
        "distance_meters": distance / 100
    })

# NOUVELLE ROUTE - À ajouter dans server_for_robot.py
@app.route('/api/debug_system', methods=['POST'])
def debug_system():
    """
    Diagnostic complet côté serveur
    """
    global raw_path_log, is_recording, current_speed, current_angle
    
    debug_info = {
        "server_state": {
            "is_recording": is_recording,
            "current_speed": current_speed,
            "current_angle": current_angle,
            "raw_events_count": len(raw_path_log),
            "timestamp": time.time()
        },
        "raw_events": [],
        "processed_vectors": [],
        "position_analysis": {}
    }
    
    # 1. Analyser les événements bruts
    print("=== DIAGNOSTIC SERVEUR ===")
    print(f"Événements bruts: {len(raw_path_log)}")
    
    for i, event in enumerate(raw_path_log):
        event_info = {
            "index": i,
            "time": event.get("time", 0),
            "action": event.get("action", "unknown"),
            "details": {k: v for k, v in event.items() if k not in ["time", "action"]}
        }
        debug_info["raw_events"].append(event_info)
        print(f"Événement {i}: {event}")
    
    # 2. Traiter en vecteurs
    try:
        processed_vectors = process_raw_path_to_vectors(raw_path_log)
        merged_vectors = merge_vectors(processed_vectors)
        debug_info["processed_vectors"] = merged_vectors
        print(f"Vecteurs traités: {len(processed_vectors)} → {len(merged_vectors)} après fusion")
        
        for i, vector in enumerate(merged_vectors):
            print(f"Vecteur {i}: {vector}")
            
    except Exception as e:
        debug_info["processing_error"] = str(e)
        print(f"ERREUR traitement vecteurs: {e}")
    
    # 3. Calculer position finale
    try:
        if debug_info["processed_vectors"]:
            final_position = calculateRealPositionFromVectors(debug_info["processed_vectors"])
            distance_to_start = math.sqrt(final_position['x']**2 + final_position['y']**2)
            
            debug_info["position_analysis"] = {
                "final_position": final_position,
                "distance_to_start_cm": distance_to_start,
                "distance_to_start_m": distance_to_start / 100
            }
            
            print(f"Position finale: {final_position}")
            print(f"Distance au départ: {distance_to_start:.1f}cm")
            
    except Exception as e:
        debug_info["position_error"] = str(e)
        print(f"ERREUR calcul position: {e}")
    
    print("========================")
    
    return jsonify(debug_info)

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

@app.route('/api/get_current_position', methods=['POST'])
def get_current_position():
    """
    Calcule la position réelle actuelle du robot basée sur l'enregistrement côté serveur
    Cette fonction est la référence absolue pour la position
    """
    global raw_path_log, is_recording, current_speed, current_angle
    
    if not is_recording:
        return jsonify({
            "status": "error", 
            "message": "Aucun enregistrement en cours"
        }), 400
    
    try:
        # 1. Traiter les événements bruts en vecteurs
        print("=== CALCUL POSITION SERVEUR ===")
        current_vectors = merge_vectors(process_raw_path_to_vectors(raw_path_log))
        print(f"Vecteurs traités: {len(current_vectors)}")
        
        # 2. Calculer la position finale précise
        position = calculateRealPositionFromVectors(current_vectors)
        
        # 3. Calculer la distance euclidienne au point de départ
        distance_to_start = math.sqrt(position['x']**2 + position['y']**2)
        
        # 4. Calculer les mouvements nécessaires pour la fermeture
        closure_movements = _calculate_closure_movements(position, current_speed, current_angle)
        
        print(f"Position finale: x={position['x']:.1f}cm, y={position['y']:.1f}cm, angle={position['angle']:.1f}°")
        print(f"Distance au départ: {distance_to_start:.1f}cm")
        print("==============================")
        
        return jsonify({
            "status": "success",
            "position": position,
            "distance_to_start": distance_to_start,
            "closure_movements": closure_movements,
            "vectors_count": len(current_vectors)
        })
        
    except Exception as e:
        print(f"Erreur calcul position: {e}")
        return jsonify({
            "status": "error", 
            "message": f"Erreur calcul position: {str(e)}"
        }), 500

def _calculate_closure_movements(current_position, speed_kmh, turn_angle):
    """
    Calcule les mouvements nécessaires pour fermer la zone
    
    Args:
        current_position (dict): Position actuelle {x, y, angle}
        speed_kmh (float): Vitesse actuelle
        turn_angle (float): Angle de rotation configuré
        
    Returns:
        dict: Mouvements de fermeture calculés
    """
    start_pos = {'x': 0, 'y': 0}
    
    # Distance au départ
    dx = start_pos['x'] - current_position['x']
    dy = start_pos['y'] - current_position['y']
    distance_to_start = math.sqrt(dx*dx + dy*dy)
    
    movements = {
        "rotation": None,
        "forward": None
    }
    
    if distance_to_start > 10:  # Plus de 10cm
        # Calculer l'angle de retour
        target_angle = math.degrees(math.atan2(dy, dx))
        current_angle = current_position['angle']
        
        # Normaliser les angles
        while target_angle <= -180: target_angle += 360
        while target_angle > 180: target_angle -= 360
        while current_angle <= -180: current_angle += 360
        while current_angle > 180: current_angle -= 360
        
        relative_angle = target_angle - current_angle
        while relative_angle <= -180: relative_angle += 360
        while relative_angle > 180: relative_angle -= 360
        
        # Calculer le nombre de rotations nécessaires
        if abs(relative_angle) > 5:
            rotations_needed = max(1, int(abs(relative_angle) / turn_angle))
            movements["rotation"] = {
                "angle": relative_angle,
                "steps": rotations_needed,
                "direction": "right" if relative_angle > 0 else "left"
            }
        
        # Calculer le mouvement de fermeture
        speed_cm_per_sec = (speed_kmh * 100000) / 3600
        duration_ms = int((distance_to_start / speed_cm_per_sec) * 1000) if speed_cm_per_sec > 0 else 2000
        
        movements["forward"] = {
            "distance": distance_to_start,
            "duration": duration_ms
        }
    
    return movements

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

@app.route('/api/undo_last_segment', methods=['POST'])
def undo_last_segment():
    """
    Annule le dernier segment enregistré côté serveur
    """
    global raw_path_log, is_recording
    
    if not is_recording:
        return jsonify({
            "status": "error", 
            "message": "Aucun enregistrement en cours"
        }), 400
    
    if len(raw_path_log) <= 1:  # Garder au minimum l'événement "start"
        return jsonify({
            "status": "error", 
            "message": "Aucun segment à annuler"
        }), 400
    
    try:
        # Trouver le dernier événement significatif à supprimer
        events_removed = []
        
        # Supprimer les événements depuis la fin jusqu'à trouver un mouvement/rotation
        while len(raw_path_log) > 1:
            last_event = raw_path_log.pop()
            events_removed.append(last_event)
            
            # Si c'est un mouvement ou une rotation, on s'arrête
            if last_event.get("action") in ["forward", "backward", "turn_left", "turn_right"]:
                break
            
            # Si c'est un STOP, continuer pour trouver le mouvement associé
            if last_event.get("action") == "stop":
                continue
        
        # Recalculer les vecteurs après suppression
        updated_vectors = merge_vectors(process_raw_path_to_vectors(raw_path_log))
        
        print(f"UNDO: Supprimé {len(events_removed)} événement(s)")
        print(f"UNDO: {len(updated_vectors)} vecteurs restants")
        
        return jsonify({
            "status": "success",
            "message": f"Dernier segment annulé ({len(events_removed)} événement(s) supprimé(s))",
            "remaining_vectors": len(updated_vectors)
        })
        
    except Exception as e:
        print(f"Erreur lors de l'annulation: {e}")
        return jsonify({
            "status": "error", 
            "message": f"Erreur interne: {str(e)}"
        }), 500
   

@app.route('/api/mark_point', methods=['POST'])
def mark_point():
    """
    Marque un point d'intérêt dans l'enregistrement
    Équivalent à un STOP mais avec annotation spéciale
    
    Returns:
        JSON: Confirmation du marquage
    """
    global raw_path_log, is_recording
    
    if not is_recording:
        return jsonify({
            "status": "error", 
            "message": "Aucun enregistrement en cours"
        }), 400
    
    try:
        data = request.json or {}
        
        # Ajouter un événement de marquage dans le log
        marker_event = {
            "time": time.time(),
            "action": "mark_point",
            "note": data.get("note", "Point marqué"),
            "timestamp": data.get("timestamp", int(time.time() * 1000))
        }
        
        raw_path_log.append(marker_event)
        
        print(f"POINT MARQUÉ: {marker_event['note']} à {len(raw_path_log)} événements")
        
        return jsonify({
            "status": "success",
            "message": f"Point marqué avec succès",
            "total_events": len(raw_path_log)
        })
        
    except Exception as e:
        print(f"Erreur marquage point: {e}")
        return jsonify({
            "status": "error", 
            "message": f"Erreur: {str(e)}"
        }), 500

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

@app.route('/api/generate_advanced_path/<string:zone_name>', methods=['POST'])
def generate_advanced_path(zone_name):
    """
    Génère un parcours de tonte avancé avec périmètre, passes concentriques et centre parallèle
    
    Args:
        zone_name (str): Nom de la zone pour laquelle générer le parcours
        
    Body JSON attendu:
        - width (float): Largeur de coupe en cm
        - concentric_passes (int): Nombre de passes concentriques (optionnel, défaut 3)
        
    Returns:
        JSON: Statut et message de la génération
    """
    try:
        # Récupérer les paramètres de la requête
        data = request.json or {}
        mower_width = data.get('width')
        concentric_passes = data.get('concentric_passes', 3)
        
        # Validation des paramètres
        if not mower_width or not isinstance(mower_width, (int, float)):
            return jsonify({
                "status": "error", 
                "message": "Largeur de coupe non spécifiée ou invalide."
            }), 400
            
        if not isinstance(concentric_passes, int) or concentric_passes < 1:
            concentric_passes = 3
            
        mower_width = float(mower_width)
        
        # Récupérer les vecteurs de la zone
        perimeter_vectors = db_manager.get_zone_by_name(zone_name)
        if not perimeter_vectors:
            return jsonify({
                "status": "error", 
                "message": f"Zone '{zone_name}' non trouvée."
            }), 404
        
        print(f"Génération parcours avancé pour '{zone_name}': largeur={mower_width}cm, passes={concentric_passes}")
        
        # Générer le parcours avancé
        advanced_path_vectors = generate_advanced_mowing_path(
            perimeter_vectors, 
            mower_width, 
            concentric_passes
        )
        
        if not advanced_path_vectors:
            return jsonify({
                "status": "error", 
                "message": "Impossible de générer un parcours avancé pour cette zone."
            }), 500
        
        # Calculer les statistiques
        total_distance = sum(v.get('distance', 0) for v in advanced_path_vectors) / 100  # en mètres
        total_rotations = sum(1 for v in advanced_path_vectors if v.get('relative_angle', 0) != 0)
        
        # Sauvegarder le parcours
        new_zone_name = f"{zone_name}_parcours_avance"
        if db_manager.save_zone(new_zone_name, advanced_path_vectors):
            return jsonify({
                "status": "success",
                "message": f"Parcours avancé '{new_zone_name}' généré avec succès.",
                "details": f"Distance totale: {total_distance:.1f}m, {total_rotations} rotations, {len(advanced_path_vectors)} segments"
            })
        else:
            return jsonify({
                "status": "error", 
                "message": f"Le parcours '{new_zone_name}' existe déjà. Supprimez-le d'abord ou choisissez un autre nom."
            }), 409
            
    except Exception as e:
        print(f"Erreur génération parcours avancé: {e}")
        return jsonify({
            "status": "error", 
            "message": f"Erreur interne lors de la génération: {str(e)}"
        }), 500

# --- Fonctions avancées pour la génération de parcours ---
def generate_advanced_mowing_path(perimeter_vectors, mower_width_cm, concentric_passes=3):
    """
    CORRECTION COMPLÈTE: Génère un parcours de tonte optimisé
    Génère: 1) Périmètre réel, 2) Passes concentriques, 3) Lignes parallèles au centre
    """
    if not perimeter_vectors:
        print("ERREUR: Aucun vecteur de périmètre fourni")
        return []
    
    print(f"=== GÉNÉRATION PARCOURS AVANCÉ ===")
    print(f"Périmètre: {len(perimeter_vectors)} vecteurs")
    print(f"Largeur coupe: {mower_width_cm}cm")
    print(f"Passes concentriques: {concentric_passes}")
    
    all_vectors = []
    
    try:
        # PHASE 1: PÉRIMÈTRE EXACT (toujours en premier)
        print("\n--- PHASE 1: PÉRIMÈTRE ---")
        all_vectors.extend(perimeter_vectors)
        print(f"Périmètre ajouté: {len(perimeter_vectors)} vecteurs")
        
        # PHASE 2: PASSES CONCENTRIQUES (si Shapely disponible)
        if SHAPELY_AVAILABLE and concentric_passes > 0:
            print("\n--- PHASE 2: PASSES CONCENTRIQUES ---")
            
            try:
                # Convertir en points et créer le polygone
                polygon_points = _vectors_to_polygon_points(perimeter_vectors)
                print(f"Points du polygone: {len(polygon_points)}")
                
                if len(polygon_points) >= 4:
                    # Fermer le polygone automatiquement
                    if polygon_points[0] != polygon_points[-1]:
                        polygon_points.append(polygon_points[0])
                    
                    # Créer le polygone Shapely
                    main_polygon = Polygon(polygon_points)
                    
                    # Vérifier et corriger la validité
                    if not main_polygon.is_valid:
                        print("Correction du polygone invalide...")
                        main_polygon = main_polygon.buffer(0)
                    
                    if main_polygon.is_valid and not main_polygon.is_empty:
                        print(f"Polygone valide créé (aire: {main_polygon.area:.1f}cm²)")
                        
                        # Générer les passes concentriques
                        current_polygon = main_polygon
                        concentric_vectors = []
                        
                        for pass_num in range(concentric_passes):
                            print(f"\nPasse concentrique {pass_num + 1}:")
                            
                            # Distance de rétrécissement progressive
                            shrink_distance = mower_width_cm * (pass_num + 1) * 0.85
                            
                            try:
                                # CORRECTION CRITIQUE: Buffer négatif pour rétrécir
                                inner_polygon = current_polygon.buffer(-shrink_distance)
                                
                                print(f"  Buffer: -{shrink_distance:.1f}cm")
                                
                                # Vérifier si le polygone rétréci est valide
                                if inner_polygon.is_empty:
                                    print(f"  → Polygone vide après rétrécissement")
                                    break
                                
                                # Gérer les multi-polygones (fragmentation)
                                if hasattr(inner_polygon, 'geoms') and len(inner_polygon.geoms) > 1:
                                    # Prendre le plus grand fragment
                                    largest_area = 0
                                    largest_poly = None
                                    for geom in inner_polygon.geoms:
                                        if hasattr(geom, 'area') and geom.area > largest_area:
                                            largest_area = geom.area
                                            largest_poly = geom
                                    
                                    if largest_poly and largest_area > (mower_width_cm * 2) ** 2:
                                        inner_polygon = largest_poly
                                        print(f"  → Fragment principal sélectionné ({largest_area:.1f}cm²)")
                                    else:
                                        print(f"  → Aucun fragment assez grand")
                                        break
                                
                                # Vérifier l'aire minimale
                                min_area = (mower_width_cm * 2) ** 2
                                if inner_polygon.area < min_area:
                                    print(f"  → Aire trop petite ({inner_polygon.area:.1f} < {min_area:.1f}cm²)")
                                    break
                                
                                # Extraire le contour externe
                                if hasattr(inner_polygon, 'exterior') and inner_polygon.exterior:
                                    coords = list(inner_polygon.exterior.coords[:-1])  # Enlever le point de fermeture
                                    
                                    if len(coords) >= 3:
                                        print(f"  → {len(coords)} points de contour")
                                        
                                        # Convertir en vecteurs de mouvement
                                        # Alterner le sens (horaire/anti-horaire)
                                        if pass_num % 2 == 1:
                                            coords.reverse()
                                            print(f"  → Sens inversé (passe {pass_num + 1})")
                                        
                                        pass_vectors = _contour_to_vectors(coords)
                                        
                                        if pass_vectors:
                                            concentric_vectors.extend(pass_vectors)
                                            current_polygon = inner_polygon
                                            print(f"  → {len(pass_vectors)} vecteurs ajoutés")
                                        else:
                                            print(f"  → Aucun vecteur généré")
                                            break
                                    else:
                                        print(f"  → Pas assez de points ({len(coords)})")
                                        break
                                else:
                                    print(f"  → Pas de contour externe")
                                    break
                                    
                            except Exception as pass_error:
                                print(f"  → Erreur: {pass_error}")
                                break
                        
                        # Ajouter les vecteurs concentriques
                        if concentric_vectors:
                            all_vectors.extend(concentric_vectors)
                            print(f"\nPasses concentriques ajoutées: {len(concentric_vectors)} vecteurs")
                        else:
                            print("\nAucune passe concentrique générée")
                        
                        # PHASE 3: LIGNES PARALLÈLES AU CENTRE
                        print("\n--- PHASE 3: LIGNES PARALLÈLES ---")
                        
                        # Utiliser le polygone réduit pour les lignes parallèles
                        if (not current_polygon.is_empty and 
                            current_polygon.area > (mower_width_cm * 1.8) ** 2):
                            
                            parallel_vectors = _generate_parallel_vectors(current_polygon, mower_width_cm)
                            
                            if parallel_vectors:
                                all_vectors.extend(parallel_vectors)
                                print(f"Lignes parallèles ajoutées: {len(parallel_vectors)} vecteurs")
                            else:
                                print("Aucune ligne parallèle générée")
                        else:
                            print("Zone centrale trop petite pour lignes parallèles")
                    
                    else:
                        print("Polygone invalide ou vide")
                else:
                    print("Pas assez de points pour créer un polygone")
                    
            except Exception as shapely_error:
                print(f"Erreur Shapely: {shapely_error}")
                print("Utilisation du périmètre uniquement")
        
        else:
            if not SHAPELY_AVAILABLE:
                print("Shapely non disponible - périmètre uniquement")
            else:
                print("Aucune passe concentrique demandée")
        
        # RÉSULTAT FINAL
        total_distance = sum(v.get('distance', 0) for v in all_vectors) / 100
        total_rotations = sum(1 for v in all_vectors if v.get('relative_angle', 0) != 0)
        
        print(f"\n=== PARCOURS FINAL ===")
        print(f"Total vecteurs: {len(all_vectors)}")
        print(f"Distance totale: {total_distance:.1f}m")
        print(f"Rotations: {total_rotations}")
        
        return all_vectors if all_vectors else perimeter_vectors
        
    except Exception as e:
        print(f"Erreur génération globale: {e}")
        return perimeter_vectors

def _contour_to_vectors(coords):
    """
    NOUVELLE FONCTION: Convertit une liste de coordonnées en vecteurs de mouvement
    """
    if len(coords) < 3:
        return []
    
    vectors = []
    current_angle = -90  # Angle initial du robot
    
    # Partir de la première coordonnée
    start_coord = coords[0]
    
    for i in range(1, len(coords)):
        current_coord = coords[i]
        
        # Calculer direction et distance
        dx = current_coord[0] - coords[i-1][0]
        dy = current_coord[1] - coords[i-1][1]
        
        if abs(dx) > 0.1 or abs(dy) > 0.1:  # Ignorer les micro-mouvements
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.degrees(math.atan2(dy, dx))
            
            # Calculer l'angle relatif
            relative_angle = target_angle - current_angle
            
            # Normaliser l'angle (-180 à 180)
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle <= -180:
                relative_angle += 360
            
            # Ajouter rotation si significative
            if abs(relative_angle) > 2:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
            
            # Ajouter mouvement
            vectors.append({"distance": distance, "relative_angle": 0})
            
            current_angle = target_angle
    
    # Fermer le contour (retour au point de départ)
    last_coord = coords[-1]
    first_coord = coords[0]
    
    dx = first_coord[0] - last_coord[0]
    dy = first_coord[1] - last_coord[1]
    distance_to_start = math.sqrt(dx*dx + dy*dy)
    
    if distance_to_start > 5:  # Plus de 5cm
        target_angle = math.degrees(math.atan2(dy, dx))
        relative_angle = target_angle - current_angle
        
        # Normaliser
        while relative_angle > 180:
            relative_angle -= 360
        while relative_angle <= -180:
            relative_angle += 360
        
        if abs(relative_angle) > 2:
            vectors.append({"distance": 0, "relative_angle": relative_angle})
        
        vectors.append({"distance": distance_to_start, "relative_angle": 0})
    
    print(f"  _contour_to_vectors: {len(coords)} coords → {len(vectors)} vecteurs")
    return vectors

def _generate_parallel_vectors(polygon, mower_width):
    """
    NOUVELLE FONCTION: Génère des vecteurs pour lignes parallèles
    """
    if not SHAPELY_AVAILABLE or polygon.is_empty:
        return []
    
    try:
        from shapely.geometry import LineString
        
        # Limites du polygone
        minx, miny, maxx, maxy = polygon.bounds
        
        # Générer waypoints parallèles
        waypoints = []
        y = miny + mower_width / 2
        direction = 1  # 1 = gauche→droite, -1 = droite→gauche
        
        while y < maxy - mower_width / 2:
            # Ligne horizontale à la hauteur y
            line = LineString([(minx - mower_width, y), (maxx + mower_width, y)])
            intersection = polygon.intersection(line)
            
            if not intersection.is_empty:
                # Traiter les segments d'intersection
                segments = []
                if hasattr(intersection, 'geoms'):
                    segments = [geom for geom in intersection.geoms if hasattr(geom, 'coords')]
                elif hasattr(intersection, 'coords'):
                    segments = [intersection]
                
                # Prendre le segment le plus long
                longest_segment = None
                max_length = 0
                
                for segment in segments:
                    try:
                        coords = list(segment.coords)
                        if len(coords) >= 2:
                            length = math.sqrt(
                                (coords[-1][0] - coords[0][0])**2 + 
                                (coords[-1][1] - coords[0][1])**2
                            )
                            if length > max_length:
                                max_length = length
                                longest_segment = coords
                    except:
                        continue
                
                # Ajouter le segment le plus long
                if longest_segment and max_length > mower_width:
                    if direction == 1:
                        waypoints.extend([
                            {'x': longest_segment[0][0], 'y': longest_segment[0][1]},
                            {'x': longest_segment[-1][0], 'y': longest_segment[-1][1]}
                        ])
                    else:
                        waypoints.extend([
                            {'x': longest_segment[-1][0], 'y': longest_segment[-1][1]},
                            {'x': longest_segment[0][0], 'y': longest_segment[0][1]}
                        ])
            
            y += mower_width * 0.8  # 80% de recouvrement
            direction *= -1  # Alterner la direction
        
        # Convertir waypoints en vecteurs
        if waypoints:
            return _path_to_vectors(waypoints)
        
        return []
        
    except Exception as e:
        print(f"Erreur génération parallèles: {e}")
        return []

def _convert_vectors_to_waypoints(vectors):
    """
    NOUVELLE FONCTION: Convertit les vecteurs de mouvement en waypoints
    Cette fonction suit exactement le tracé enregistré
    """
    if not vectors:
        return []
    
    waypoints = []
    x, y, angle = 0, 0, -90.0
    
    # Point de départ
    waypoints.append({'x': x, 'y': y})
    
    for vector in vectors:
        # Appliquer la rotation si présente
        if vector.get('relative_angle', 0) != 0:
            angle += vector['relative_angle']
            # Pas de waypoint pour les rotations pures
        
        # Appliquer le mouvement si présent
        distance = vector.get('distance', 0)
        if distance > 0:
            angle_rad = math.radians(angle)
            new_x = x + distance * math.cos(angle_rad)
            new_y = y + distance * math.sin(angle_rad)
            waypoints.append({'x': new_x, 'y': new_y})
            x, y = new_x, new_y
    
    print(f"_convert_vectors_to_waypoints: {len(waypoints)} waypoints générés")
    return waypoints

def _generate_perimeter_path_corrected(coords, offset_distance, reverse=False):
    """
    CORRECTION: Génère un chemin périmétrique plus robuste
    """
    if len(coords) < 3:
        print(f"_generate_perimeter_path_corrected: Pas assez de coordonnées ({len(coords)})")
        return []
    
    waypoints = []
    coords_list = list(coords)
    
    if reverse:
        coords_list.reverse()
    
    # Convertir chaque coordonnée en waypoint
    for i, coord in enumerate(coords_list):
        try:
            if hasattr(coord, '__iter__') and len(coord) >= 2:
                x, y = float(coord[0]), float(coord[1])
                waypoints.append({'x': x, 'y': y})
            else:
                print(f"Coordonnée invalide ignorée à l'index {i}: {coord}")
        except (ValueError, TypeError) as e:
            print(f"Erreur conversion coordonnée {i}: {e}")
    
    print(f"_generate_perimeter_path_corrected: {len(waypoints)} waypoints générés")
    return waypoints

def _generate_parallel_path_for_polygon_corrected(polygon, mower_width):
    """
    CORRECTION: Génère un parcours parallèle plus robuste pour le centre uniquement
    """
    if not SHAPELY_AVAILABLE or polygon.is_empty:
        return []
    
    try:
        from shapely.geometry import LineString
        
        # Récupérer les limites du polygone
        minx, miny, maxx, maxy = polygon.bounds
        
        waypoints = []
        y = miny + mower_width / 2
        direction = 1  # 1 pour gauche à droite, -1 pour droite à gauche
        line_count = 0
        
        while y < maxy and line_count < 100:  # Limite de sécurité
            try:
                # Créer une ligne horizontale
                line = LineString([(minx - mower_width, y), (maxx + mower_width, y)])
                intersection = polygon.intersection(line)
                
                if intersection.is_empty:
                    y += mower_width * 0.9
                    direction *= -1
                    continue
                
                # Traiter les intersections
                segments = []
                if hasattr(intersection, 'geoms'):
                    segments = [geom for geom in intersection.geoms if hasattr(geom, 'coords')]
                elif hasattr(intersection, 'coords'):
                    segments = [intersection]
                
                # Ajouter les waypoints
                for segment in segments:
                    try:
                        coords = list(segment.coords)
                        if len(coords) >= 2:
                            start_point = {'x': coords[0][0], 'y': coords[0][1]}
                            end_point = {'x': coords[-1][0], 'y': coords[-1][1]}
                            
                            if direction == 1:
                                waypoints.extend([start_point, end_point])
                            else:
                                waypoints.extend([end_point, start_point])
                                
                    except Exception as segment_error:
                        print(f"Erreur segment: {segment_error}")
                
                y += mower_width * 0.8  # 80% de recouvrement
                direction *= -1
                line_count += 1
                
            except Exception as line_error:
                print(f"Erreur ligne: {line_error}")
                y += mower_width
                line_count += 1
        
        print(f"Tonte parallèle: {len(waypoints)} waypoints, {line_count} lignes")
        return waypoints
        
    except Exception as e:
        print(f"Erreur tonte parallèle: {e}")
        return []

"""Permettre l'arrêt explicite de la tonte en cours :"""

@app.route('/api/stop_autonomous', methods=['POST'])
def stop_autonomous_mowing():
    """Arrête la tonte autonome en cours"""
    global robot_status
    
    if "tonte" in robot_status.lower():
        robot_status = "Arrêt demandé"
        robot.stop()  # Arrêter les moteurs
        return jsonify({"status": "success", "message": "Arrêt de la tonte demandé"})
    else:
        return jsonify({"status": "error", "message": "Aucune tonte en cours"})        

@app.route('/api/delete_zone/<int:zone_id>', methods=['DELETE'])
def delete_zone(zone_id):
    """
    Supprime une zone spécifique par son ID.
    
    Args:
        zone_id (int): L'ID de la zone à supprimer
        
    Returns:
        JSON: Résultat de l'opération
    """
    # Vérifier que le robot n'est pas en cours de tonte
    if robot_status != "Prêt" and "tonte" in robot_status.lower():
        return jsonify({
            "status": "error", 
            "message": "Impossible de supprimer une zone pendant la tonte."
        }), 409
    
    success = db_manager.delete_zone(zone_id)
    
    if success:
        return jsonify({
            "status": "success",
            "message": f"Zone supprimée avec succès."
        })
    else:
        return jsonify({
            "status": "error",
            "message": "Erreur lors de la suppression de la zone."
        }), 404

@app.route('/api/delete_zones', methods=['DELETE'])
def delete_multiple_zones():
    """
    Supprime plusieurs zones en une fois.
    
    Body JSON attendu:
        - zone_ids (list): Liste des IDs des zones à supprimer
        
    Returns:
        JSON: Résultat détaillé de l'opération
    """
    # Vérifier que le robot n'est pas en cours de tonte
    if robot_status != "Prêt" and "tonte" in robot_status.lower():
        return jsonify({
            "status": "error", 
            "message": "Impossible de supprimer des zones pendant la tonte."
        }), 409
    
    data = request.json or {}
    zone_ids = data.get('zone_ids', [])
    
    if not zone_ids or not isinstance(zone_ids, list):
        return jsonify({
            "status": "error",
            "message": "Liste d'IDs de zones requise."
        }), 400
    
    # Valider que tous les IDs sont des entiers
    try:
        zone_ids = [int(id) for id in zone_ids]
    except (ValueError, TypeError):
        return jsonify({
            "status": "error",
            "message": "Tous les IDs doivent être des nombres entiers."
        }), 400
    
    result = db_manager.delete_zones_by_ids(zone_ids)
    
    return jsonify({
        "status": "success" if result["success"] else "partial",
        "message": result["message"],
        "details": {
            "deleted": result["success"],
            "failed": result["failed"]
        }
    })

@app.route('/api/delete_all_zones', methods=['DELETE'])
def delete_all_zones():
    """
    Supprime TOUTES les zones. 
    Opération dangereuse qui nécessite une confirmation explicite.
    
    Body JSON attendu:
        - confirm (bool): Doit être True pour confirmer l'opération
        
    Returns:
        JSON: Résultat de l'opération avec statistiques
    """
    # Vérifier que le robot n'est pas en cours de tonte
    if robot_status != "Prêt" and "tonte" in robot_status.lower():
        return jsonify({
            "status": "error", 
            "message": "Impossible de supprimer les zones pendant la tonte."
        }), 409
    
    data = request.json or {}
    confirmation = data.get('confirm', False)
    
    if not confirmation:
        return jsonify({
            "status": "error",
            "message": "Confirmation requise pour supprimer toutes les zones."
        }), 400
    
    result = db_manager.delete_all_zones()
    
    if result["success"]:
        return jsonify({
            "status": "success",
            "message": result["message"],
            "statistics": {
                "zones_deleted": result["zones_deleted"],
                "history_deleted": result["history_deleted"]
            }
        })
    else:
        return jsonify({
            "status": "error",
            "message": result["message"]
        }), 500        

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)