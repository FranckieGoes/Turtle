# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify
from robot_controller_class import RobotController
import time
import json
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
# Un bloc try-except est utilisé pour gérer les erreurs d'initialisation GPIO
# si le script est exécuté sur une machine sans broches physiques.
try:
    robot = RobotController(motor_gauche_pins, motor_droit_pins)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    robot = None # Définissez robot sur None pour gérer le cas d'échec

# Variables globales pour l'état du robot et l'enregistrement
# Ces valeurs sont utilisées comme état par défaut et peuvent être mises à jour via l'API.
current_angle = 5
current_speed = 50
is_recording = False
mowing_vectors = []
MOWING_DATA_FILE = 'mowing_data.json'

def load_mowing_vectors():
    """Charge les vecteurs de tonte à partir d'un fichier JSON."""
    if os.path.exists(MOWING_DATA_FILE):
        try:
            with open(MOWING_DATA_FILE, 'r') as f:
                return json.load(f)
        except (IOError, json.JSONDecodeError) as e:
            print(f"Erreur lors du chargement des vecteurs : {e}")
            return []
    return []

def save_mowing_vectors():
    """Sauvegarde les vecteurs de tonte dans un fichier JSON."""
    try:
        with open(MOWING_DATA_FILE, 'w') as f:
            json.dump(mowing_vectors, f, indent=4)
        print(f"Vecteurs de tonte sauvegardés dans {MOWING_DATA_FILE}")
    except IOError as e:
        print(f"Erreur lors de la sauvegarde des vecteurs : {e}")

# Charge les vecteurs au démarrage du serveur
mowing_vectors = load_mowing_vectors()

@app.route("/")
def index():
    """Route principale pour servir le fichier HTML de l'interface."""
    return render_template("robot_controller.html")

@app.route("/mowing_zone")
def mowing_zone():
    """Route pour servir le fichier HTML de la visualisation de la zone de tonte."""
    return render_template("mowing_zone.html")

@app.route("/api/mowing_data", methods=["GET"])
def get_mowing_data():
    """API pour fournir les données de la zone de tonte."""
    return jsonify(mowing_vectors)

@app.route("/api/command", methods=["POST"])
def handle_command():
    global current_angle, current_speed, is_recording, mowing_vectors
    
    status = "ok"
    try:
        data = request.get_json()
        command = data.get("command")
        
        # Mise à jour des variables globales si présentes dans le payload
        if "angle" in data:
            current_angle = int(data["angle"])
        if "speed" in data:
            current_speed = int(data["speed"])
        
        print(f"Commande reçue: {command}")
        
        if robot is None:
            response_message = "Erreur: Le contrôleur de robot n'est pas initialisé."
            status = "error"
        else:
            if command == "forward":
                print("Action: Avancer")
                robot.move_forward(direction=True, speed_kmh=current_speed)
                if is_recording:
                    # Ajoute un vecteur de mouvement (distance, angle relatif 0)
                    mowing_vectors.append({"distance": current_speed * 10, "relative_angle": 0})
                response_message = "Avancer"
            elif command == "backward":
                print("Action: Reculer")
                robot.move_backward(direction=False, speed_kmh=current_speed)
                if is_recording:
                    # Ajoute un vecteur de mouvement (distance, angle relatif 0)
                    mowing_vectors.append({"distance": -current_speed * 10, "relative_angle": 0})
                response_message = "Reculer"
            elif command == "turn_left":
                print(f"Action: Tourner à gauche de {current_angle}°")
                robot.turn_left(direction=True, speed_kmh=current_speed, angle_deg=current_angle)
                if is_recording:
                    # Ajoute un vecteur de rotation (distance 0, angle relatif)
                    mowing_vectors.append({"distance": 0, "relative_angle": -current_angle})
                response_message = f"Tourner à gauche de {current_angle}°"
            elif command == "turn_right":
                print(f"Action: Tourner à droite de {current_angle}°")
                robot.turn_right(direction=False, speed_kmh=current_speed, angle_deg=current_angle)
                if is_recording:
                    # Ajoute un vecteur de rotation (distance 0, angle relatif)
                    mowing_vectors.append({"distance": 0, "relative_angle": current_angle})
                response_message = f"Tourner à droite de {current_angle}°"
            elif command == "stop":
                print("Action: Arrêter le mouvement")
                robot.stop()
                response_message = "Arrêter le mouvement"
            elif command == "pause":
                print("Action: Pause")
                robot.pause()
                response_message = "Mettre en pause"
            elif command == "resume":
                print("Action: Reprendre")
                robot.resume()
                response_message = "Reprendre le mouvement"
            elif command == "set_angle":
                print(f"Nouvel angle défini à {current_angle}°")
                response_message = f"Angle mis à jour à {current_angle}°"
            elif command == "set_speed":
                print(f"Nouvelle vitesse définie à {current_speed} KM/H")
                # Appeler la nouvelle méthode pour mettre à jour la vitesse des moteurs
                robot.update_speed(current_speed)
                response_message = f"Vitesse mise à jour à {current_speed} KM/H"
            elif command == "start_recording":
                is_recording = True
                mowing_vectors = []  # Réinitialiser la zone
                print("Action: Démarrer l'enregistrement de la zone de tonte")
                response_message = "Enregistrement démarré"
            elif command == "stop_recording":
                is_recording = False
                save_mowing_vectors() # Sauvegarder les données
                print("Action: Arrêter l'enregistrement de la zone de tonte et sauvegarder le parcours")
                response_message = "Enregistrement arrêté et parcours sauvegardé"
            else:
                print(f"Commande inconnue: {command}")
                response_message = f"Commande inconnue: {command}"
                status = "error"
                
    except Exception as e:
        print(f"Erreur lors du traitement de la commande : {e}")
        response_message = f"Erreur du serveur : {e}"
        status = "error"

    response_data = {
        "status": status,
        "message": f"Commande '{command}' reçue et traitée. Action: {response_message}"
    }
    return jsonify(response_data)

if __name__ == '__main__':
    # La ligne ci-dessous est pour le développement, pour le déploiement sur Raspberry Pi,
    # il est préférable d'utiliser un serveur WSGI comme Gunicorn
    # app.run(host='0.0.0.0', port=5000, debug=True)
    app.run(host='0.0.0.0', port=5000)
