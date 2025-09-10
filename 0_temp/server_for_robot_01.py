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

# Variables globales pour l'angle et la vitesse
# Ces valeurs sont utilisées comme état par défaut et peuvent être mises à jour via l'API.
current_angle = 0
current_speed = 3

# Variables pour l'enregistrement de la zone
is_recording = False
recorded_path = []
MOWING_ZONE_FILE = 'mowing_zone.json'

@app.route("/")
def index():
    """Route principale pour servir le fichier HTML de l'interface."""
    return render_template("robot_controller.html")

@app.route("/mowing_zone")
def mowing_zone():
    """Route pour servir le fichier HTML de la visualisation de la zone de tonte."""
    return render_template("mowing_zone.html")

@app.route("/api/control", methods=["POST"])
def control():
    """Endpoint API pour recevoir et traiter les commandes du robot."""
    global current_angle, current_speed, is_recording, recorded_path
    
    data = request.get_json()
    command = data.get("command")
    status = "success"
    response_message = ""

    print(f"Commande reçue : {command}")

    # Enregistrement des commandes si le mode enregistrement est actif
    if is_recording:
        if command == "forward":
            recorded_path.append({"command": "forward", "speed": current_speed})
        elif command == "backward":
            recorded_path.append({"command": "backward", "speed": current_speed})
        elif command == "left":
            recorded_path.append({"command": "turn", "direction": "left", "angle": current_angle, "speed": current_speed})
        elif command == "right":
            recorded_path.append({"command": "turn", "direction": "right", "angle": current_angle, "speed": current_speed})
        elif command == "set_speed":
            recorded_path.append({"command": "set_speed", "speed": data.get("value")})
        elif command == "set_angle":
            recorded_path.append({"command": "set_angle", "angle": data.get("value")})
    
    # Traitement des commandes
    if robot is None:
        response_message = "Erreur: Le contrôleur du robot n'est pas initialisé."
        status = "error"
    elif command == "forward":
        print(f"Action: Avancer à {current_speed} KM/H")
        if robot.is_moving:
            response_message = "Le robot est déjà en mouvement."
        else:
            robot.start_continuous_move(current_speed, "forward")
            response_message = "Démarrer le mouvement vers l'avant"
    elif command == "backward":
        print(f"Action: Reculer à {current_speed} KM/H")
        if robot.is_moving:
            response_message = "Le robot est déjà en mouvement."
        else:
            robot.start_continuous_move(current_speed, "backward")
            response_message = "Démarrer le mouvement vers l'arrière"
    elif command == "left":
        print(f"Action: Tourner à gauche de {current_angle}° à {current_speed} KM/H")
        robot.turn_on_spot(current_speed, "left", current_angle)
        response_message = "Tourner à gauche"
    elif command == "right":
        print(f"Action: Tourner à droite de {current_angle}° à {current_speed} KM/H")
        robot.turn_on_spot(current_speed, "right", current_angle)
        response_message = "Tourner à droite"
    elif command == "stop":
        print("Action: Arrêter")
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
        value = data.get("value")
        current_angle = value
        print(f"Nouvel angle défini à {current_angle}°")
        response_message = f"Angle mis à jour à {current_angle}°"
    elif command == "set_speed":
        value = data.get("value")
        current_speed = value
        print(f"Nouvelle vitesse définie à {current_speed} KM/H")
        robot.update_speed(current_speed)
        response_message = f"Vitesse mise à jour à {current_speed} KM/H"
    elif command == "start_recording":
        print("Action: Démarrer l'enregistrement")
        is_recording = True
        recorded_path = [] # Réinitialiser le chemin
        response_message = "Début de l'enregistrement de la zone"
    elif command == "stop_recording":
        print("Action: Arrêter l'enregistrement")
        is_recording = False
        response_message = "Fin de l'enregistrement de la zone"
    elif command == "execute_zone":
        print("Action: Exécuter la zone de tonte enregistrée")
        response_message = "Exécution de la zone de tonte"
        # Ajoutez la logique d'exécution ici si nécessaire
        # Par exemple, une boucle qui parcourt recorded_path et appelle les méthodes du robot
    else:
        print(f"Commande inconnue: {command}")
        response_message = f"Commande inconnue: {command}"
        status = "error"

    response_data = {
        "status": status,
        "message": f"Commande '{command}' reçue et traitée. Action: {response_message}"
    }
    return jsonify(response_data)

@app.route("/api/mowing_zone", methods=["GET"])
def get_mowing_zone():
    """Endpoint API pour récupérer la zone de tonte enregistrée."""
    global recorded_path
    
    # Conversion du chemin en format utilisable pour le tracé
    mowing_vectors = []
    
    # Pour simplifier, on convertit les commandes en vecteurs
    # Cela est une approximation et doit être affinée pour une vraie application
    for step in recorded_path:
        command = step.get('command')
        if command == "forward":
            # Approximation: Avancer de 50 unités par commande "forward"
            mowing_vectors.append({"distance": 50, "relative_angle": 0}) 
        elif command == "backward":
            # Approximation: Reculer de 50 unités par commande "backward"
            mowing_vectors.append({"distance": -50, "relative_angle": 0}) 
        elif command == "turn":
            direction = step.get('direction')
            angle = step.get('angle')
            if direction == "left":
                mowing_vectors.append({"distance": 0, "relative_angle": -angle})
            else:
                mowing_vectors.append({"distance": 0, "relative_angle": angle})

    return jsonify({"path_data": mowing_vectors})

@app.route("/api/save_mowing_zone", methods=["POST"])
def save_mowing_zone():
    """Endpoint API pour sauvegarder le chemin enregistré dans un fichier."""
    global recorded_path
    try:
        with open(MOWING_ZONE_FILE, 'w') as f:
            json.dump(recorded_path, f, indent=4)
        return jsonify({"status": "success", "message": "Zone de tonte sauvegardée."})
    except Exception as e:
        return jsonify({"status": "error", "message": f"Erreur de sauvegarde: {str(e)}"})

@app.route("/api/load_mowing_zone", methods=["GET"])
def load_mowing_zone():
    """Endpoint API pour charger un chemin enregistré depuis un fichier."""
    global recorded_path
    if os.path.exists(MOWING_ZONE_FILE):
        try:
            with open(MOWING_ZONE_FILE, 'r') as f:
                recorded_path = json.load(f)
            return jsonify({"status": "success", "message": "Zone de tonte chargée."})
        except Exception as e:
            return jsonify({"status": "error", "message": f"Erreur de chargement: {str(e)}"})
    else:
        return jsonify({"status": "error", "message": "Fichier de zone non trouvé."})


if __name__ == '__main__':
    # Le serveur web est exécuté sur l'adresse 0.0.0.0, ce qui le rend
    # accessible depuis n'importe quelle machine sur le même réseau.
    app.run(host='0.0.0.0', port=5000, debug=True)
