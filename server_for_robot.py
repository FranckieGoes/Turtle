# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify
from robot_controller_class import RobotController
import time

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
current_angle = 5
current_speed = 50

@app.route("/")
def index():
    """Route principale pour servir le fichier HTML de l'interface."""
    return render_template("robot_controller.html")

@app.route("/api/command", methods=['POST'])
def handle_command():
    """
    Gère toutes les commandes envoyées par l'interface web.
    Les commandes sont envoyées en tant que données JSON.
    """
    if not robot:
        return jsonify({"error": "RobotController non initialisé. Veuillez vérifier les connexions GPIO."}), 500

    data = request.get_json()
    command = data.get('command')
    angle = data.get('angle')
    speed = data.get('speed')
    
    global current_angle, current_speed

    print(f"Commande reçue: {command}")
    
    # Assurez-vous que les valeurs d'angle et de vitesse sont valides avant de les utiliser
    if angle is not None:
        current_angle = angle
    if speed is not None:
        current_speed = speed

    print(f"Angle actuel: {current_angle}°")
    print(f"Vitesse actuelle: {current_speed} KM/H")

    response_message = ""
    status = "success"

    if command == "forward":
        print("Action: Avancer")
        robot.move('avant', speed_kmh=current_speed, angle_deg=current_angle)
        response_message = "Avancer"
    elif command == "backward":
        print("Action: Reculer")
        robot.move('arriere', speed_kmh=current_speed, angle_deg=current_angle)
        response_message = "Reculer"
    elif command == "left":
        print("Action: Gauche")
        robot.turn('gauche', speed_kmh=current_speed, angle_deg=current_angle)
        response_message = "Tourner à gauche"
    elif command == "right":
        print("Action: Droite")
        robot.turn('droite', speed_kmh=current_speed, angle_deg=current_angle)
        response_message = "Tourner à droite"
    elif command == "stop":
        print("Action: Stop")
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
        # L'angle a déjà été mis à jour par le flux de données
        print(f"Nouvel angle défini à {current_angle}°")
        response_message = f"Angle mis à jour à {current_angle}°"
    elif command == "set_speed":
        # La vitesse a déjà été mise à jour par le flux de données
        print(f"Nouvelle vitesse définie à {current_speed} KM/H")
        # Appelez la nouvelle méthode pour mettre à jour la vitesse des moteurs
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
    finally:
        # Assurez-vous que les broches GPIO sont nettoyées en cas d'interruption
        if robot:
            robot.cleanup_gpio()