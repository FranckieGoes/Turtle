# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from flask import Flask, request, jsonify, render_template
from robot_controller_class import RobotController
import json
import os
import time

# Créez l'instance de l'application Flask
app = Flask(__name__)

# Définition des broches GPIO pour les moteurs
# Ces broches sont basées sur une configuration typique.
# Vous devez les ajuster pour correspondre à votre câblage.
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
# si le script est exécuté sur une machine sans broches physiques (comme un PC). 
try:
    robot = RobotController(motor_gauche_pins, motor_droit_pins)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    #robot = None # Définissez robot sur None pour gérer le cas d'échec
    #robot = RobotController(motor_gauche_pins)

# Variables globales pour l'angle et la vitesse
current_angle = 0
current_speed = 3

# Variables pour l'enregistrement du parcours de tonte
mowing_path = []
is_recording = False
RECORD_FILE = "mowing_path.json"

# --- Fonctions utilitaires ---

def save_path():
    """Sauvegarde le parcours enregistré dans un fichier JSON."""
    with open(RECORD_FILE, 'w') as f:
        json.dump(mowing_path, f, indent=4)
    print("Parcours sauvegardé avec succès.")

def load_path():
    """Charge le parcours depuis le fichier JSON."""
    global mowing_path
    if os.path.exists(RECORD_FILE):
        with open(RECORD_FILE, 'r') as f:
            mowing_path = json.load(f)
        print("Parcours chargé avec succès.")
    else:
        print("Le fichier de parcours n'existe pas.")
        mowing_path = []

# Charger le parcours au démarrage du serveur
load_path()

# --- Routes de l'API Flask ---

@app.route("/")
def index():
    """
    Route principale pour servir le fichier HTML de l'interface.
    Ce fichier HTML doit être placé dans un dossier 'templates'.
    """
    return render_template("robot_controller.html")

@app.route("/api/path", methods=["GET"])
def get_path():
    """
    Renvoie le parcours de tonte enregistré au format JSON.
    """
    return jsonify(mowing_path)

@app.route("/api/command", methods=["POST"])
def command():
    """
    Gère les commandes reçues de l'interface web via l'API.
    """
    global current_angle, current_speed, is_recording

    data = request.json
    command = data.get("command")
    status = "success"
    response_message = ""

    print(f"Commande reçue: {command}")

    if not robot:
        response_message = "Le contrôleur de robot n'est pas initialisé."
        status = "error"
    elif command == "move_forward":
        robot.move_forward(current_speed)
        response_message = f"Avancer avec une vitesse de {current_speed} KM/H"
        if is_recording:
            mowing_path.append({"action": "forward", "speed": current_speed, "time": time.time()})
    elif command == "move_backward":
        robot.move_backward(current_speed)
        response_message = f"Reculer avec une vitesse de {current_speed} KM/H"
        if is_recording:
            mowing_path.append({"action": "backward", "speed": current_speed, "time": time.time()})
    elif command == "turn_left":
        robot.turn_in_place("left", current_speed, current_angle)
        response_message = f"Tourner à gauche avec un angle de {current_angle}°"
        if is_recording:
            mowing_path.append({"action": "turn_left", "angle": current_angle, "time": time.time()})
    elif command == "turn_right":
        robot.turn_in_place("right", current_speed, current_angle)
        response_message = f"Tourner à droite avec un angle de {current_angle}°"
        if is_recording:
            mowing_path.append({"action": "turn_right", "angle": current_angle, "time": time.time()})
    elif command == "stop":
        robot.stop()
        response_message = "Arrêter le mouvement"
    elif command == "pause":
        robot.pause()
        response_message = "Mettre en pause"
    elif command == "resume":
        robot.resume()
        response_message = "Reprendre le mouvement"
    elif command == "set_angle":
        # L'angle a été mis à jour par l'interface web
        current_angle = data.get("value")
        print(f"Nouvel angle défini à {current_angle}°")
        response_message = f"Angle mis à jour à {current_angle}°"
    elif command == "set_speed":
        # La vitesse a été mise à jour par l'interface web
        current_speed = data.get("value")
        robot.update_speed(current_speed)
        print(f"Nouvelle vitesse définie à {current_speed} KM/H")
        response_message = f"Vitesse mise à jour à {current_speed} KM/H"
    elif command == "start_recording":
        is_recording = True
        mowing_path.clear()
        print("Début de l'enregistrement du parcours.")
        response_message = "Enregistrement démarré"
    elif command == "stop_recording":
        is_recording = False
        save_path()
        print("Enregistrement du parcours arrêté et sauvegardé.")
        response_message = "Enregistrement arrêté et sauvegardé"
    elif command == "play_recording":
        response_message = "Lecture du parcours..."
        status = "processing"
        # Démarrer la lecture du parcours dans un thread séparé pour ne pas bloquer l'API
        # (à implémenter dans un cas réel)
        print("Lecture du parcours...")
        for action in mowing_path:
            if action["action"] == "forward":
                robot.move_forward(action["speed"])
                time.sleep(1) # Simule une durée de mouvement
            elif action["action"] == "backward":
                robot.move_backward(action["speed"])
                time.sleep(1)
            elif action["action"] == "turn_left":
                robot.turn_in_place("left", action["speed"], action["angle"])
                time.sleep(1)
            elif action["action"] == "turn_right":
                robot.turn_in_place("right", action["speed"], action["angle"])
                time.sleep(1)
        robot.stop()
        response_message = "Lecture du parcours terminée"
        status = "success"
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
    # Lance le serveur Flask.
    # host='0.0.0.0' rend le serveur accessible depuis n'importe quelle adresse IP
    # sur le même réseau local, ce qui est nécessaire pour un contrôle à distance.
    # debug=True est utile pour le développement, mais doit être désactivé en production.
    print("Démarrage du serveur...")
    app.run(host='0.0.0.0', port=5000, debug=True)
