# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify
from robot_controller_class import RobotController
import time
import sqlite3
import math
import os

app = Flask(__name__)

# Broches des moteurs
motor_gauche_pins = {
    'dir_pin': 27,
    'step_pin': 17,
    'en_pin': 22
}
motor_droit_pins = {
    'dir_pin': 16,
    'step_pin': 20,
    'en_pin': 21
}

# Créez l'instance du contrôleur de robot
try:
    robot = RobotController(motor_gauche_pins, motor_droit_pins)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    robot = None

# Variables d'état du robot et de l'enregistrement
current_angle = 5
current_speed = 50
is_recording = False
robot_x = 0.0
robot_y = 0.0
robot_orientation_deg = 0.0
last_vector_angle_deg = 0.0
last_command_time = time.time()

# Définition du fichier de base de données
DB_FILE = 'mowing_zones.db'

def setup_database():
    """Crée la base de données SQLite et la table de la zone de tonte."""
    try:
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute('''
            CREATE TABLE IF NOT EXISTS mowing_path (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                distance_cm REAL,
                relative_angle_deg REAL
            )
        ''')
        conn.commit()
        conn.close()
    except sqlite3.Error as e:
        print(f"Erreur de base de données lors de la configuration: {e}")

# Assurez-vous que la base de données est prête au démarrage du serveur
setup_database()

@app.route("/")
def index():
    """Route principale pour servir le fichier HTML de l'interface."""
    return render_template("robot_controller.html")

@app.route("/visualize")
def visualize():
    """Route pour servir la page de visualisation de la zone de tonte."""
    return render_template("mowing_zone.html")

@app.route("/get_mowing_path")
def get_mowing_path():
    """Récupère et renvoie le chemin de la zone de tonte depuis la base de données."""
    try:
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("SELECT distance_cm, relative_angle_deg FROM mowing_path ORDER BY id ASC")
        path_data = c.fetchall()
        conn.close()
        # Convertir les données en une liste de dictionnaires pour le JSON
        path_vectors = [{"distance": row[0], "relative_angle": row[1]} for row in path_data]
        return jsonify(path_vectors)
    except sqlite3.Error as e:
        print(f"Erreur de base de données lors de la récupération du chemin: {e}")
        return jsonify({"status": "error", "message": "Erreur lors de la récupération des données."}), 500

def record_vector(distance_cm, relative_angle_deg):
    """Enregistre un vecteur de mouvement dans la base de données."""
    try:
        conn = sqlite3.connect(DB_FILE)
        c = conn.cursor()
        c.execute("INSERT INTO mowing_path (distance_cm, relative_angle_deg) VALUES (?, ?)", (distance_cm, relative_angle_deg))
        conn.commit()
        conn.close()
        print(f"Vecteur enregistré : distance={distance_cm:.2f} cm, angle relatif={relative_angle_deg:.2f}°")
    except sqlite3.Error as e:
        print(f"Erreur de base de données lors de l'insertion du vecteur: {e}")

@app.route("/command", methods=["POST"])
def handle_command():
    """Gère les commandes reçues de l'interface."""
    global current_angle, current_speed, is_recording, robot_x, robot_y, robot_orientation_deg, last_vector_angle_deg, last_command_time

    data = request.get_json()
    print("Data reçu : ", data)
    command = data.get('command')
    speed_kmh = data.get('speed', current_speed)
    angle_deg = data.get('angle', current_angle)
    direction = data.get('direction', 'avant')

    current_speed = speed_kmh
    current_angle = angle_deg

    print(f"Commande reçue: {command}")
    print(f"Angle actuel: {current_angle}°")
    print(f"Vitesse actuelle: {current_speed} KM/H")
    
    response_message = "Commande traitée"
    status = "success"

    # Calculer le temps écoulé depuis la dernière commande de mouvement
    time_elapsed = time.time() - last_command_time
    last_command_time = time.time()
    
    # Distance simplifiée (pour le prototype)
    # Dans une version réelle, cela devrait être basé sur des capteurs ou le nombre de pas moteurs.
    DISTANCE_PER_SECOND = 10 # 10 cm par seconde à pleine vitesse (à ajuster)
    distance_cm = (current_speed / 100) * DISTANCE_PER_SECOND * time_elapsed

    if command == "move":
        # Mettre à jour l'état de la position
        angle_rad = math.radians(robot_orientation_deg)
        if direction == 'avant':
            robot_x += distance_cm * math.cos(angle_rad)
            robot_y += distance_cm * math.sin(angle_rad)
        else: # arriere
            robot_x -= distance_cm * math.cos(angle_rad)
            robot_y -= distance_cm * math.sin(angle_rad)

        robot.move(direction, current_speed, current_angle)
        response_message = f"Avancer de {distance_cm:.2f} cm"

        if is_recording:
            record_vector(distance_cm, 0)
    
    elif command == "turn":
        # Mettre à jour l'état de l'orientation
        if direction == 'gauche':
            robot_orientation_deg -= current_angle
            relative_angle = -current_angle
        else: # droite
            robot_orientation_deg += current_angle
            relative_angle = current_angle
        
        # Le changement d'angle seul ne crée pas de mouvement,
        # on enregistre un vecteur de 0 distance avec l'angle relatif
        if is_recording:
            record_vector(0, relative_angle)
            
        robot.turn(direction, current_speed, current_angle)
        response_message = f"Tourner de {current_angle}°"

    elif command == "stop":
        # Un arrêt est une modification de l'état
        if is_recording:
             # Enregistrer un vecteur de distance nulle et d'angle nul pour marquer l'arrêt
             record_vector(0, 0)

        robot.stop()
        response_message = "Arrêter le mouvement"
    
    elif command == "start_recording":
        if is_recording:
            response_message = "L'enregistrement est déjà en cours."
        else:
            is_recording = True
            # Réinitialiser la base de données et l'état
            conn = sqlite3.connect(DB_FILE)
            c = conn.cursor()
            c.execute("DELETE FROM mowing_path")
            conn.commit()
            conn.close()
            robot_x, robot_y, robot_orientation_deg, last_vector_angle_deg = 0, 0, 0, 0
            
            response_message = "Démarrage de l'enregistrement"

    elif command == "stop_recording":
        if not is_recording:
            response_message = "Aucun enregistrement en cours."
        else:
            is_recording = False
            response_message = "Arrêt de l'enregistrement"

    elif command == "set_angle":
        # L'angle a déjà été mis à jour par le flux de données
        print(f"Nouvel angle défini à {current_angle}°")
        response_message = f"Angle mis à jour à {current_angle}°"
        
    elif command == "set_speed":
        # La vitesse a déjà été mise à jour par le flux de données
        print(f"Nouvelle vitesse définie à {current_speed} KM/H")
        robot.update_speed(current_speed)
        response_message = f"Vitesse mise à jour à {current_speed} KM/H"
        
    else:
        print(f"Commande inconnue: {command}")
        response_message = f"Commande inconnue: {command}"
        status = "error"

    response_data = {
        "status": status,
        "message": f"Commande '{command}' reçue et traitée. Action: {response_message}"
    }
    return jsonify(response_data)

if __name__ == '__main__':
    try:
        app.run(host='0.0.0.0', port=5000)
    except KeyboardInterrupt:
        print("Arrêt du serveur et nettoyage des GPIO...")
        if robot:
            robot.cleanup_gpio()