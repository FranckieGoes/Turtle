import RPi.GPIO as GPIO
from flask import Flask, render_template, request, jsonify, session
from robot_controller_class import RobotController
import time

app = Flask(__name__)
app.secret_key = "44bf7f8fd8c64e579af5f49f0bfce3e83e32cf156ac65ec16e0d9f876871a28a"

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

# Créez l'instance du contrôleur de robot pour qu'elle soit accessible aux routes Flask
try:
    robot = RobotController(motor_gauche_pins, motor_droit_pins)
except Exception as e:
    print(f"Erreur lors de l'initialisation du RobotController: {e}")
    robot = None # Définissez robot sur None pour gérer le cas d'échec

# Vitesse maximale en km/h pour la conversion
MAX_KMH = 10

@app.route("/")
def index():
    session['Vitesse_KMH'] = 5
    session['Marche'] = 0
    session['Demarrage'] = 0
    session['Tourne_Moteur'] = 0
    session['Distance_CM'] = 0 # Nouvelle variable de session pour la distance
    print('Valeur cookies',session)
    return render_template("index.html")

@app.route("/<action>")
def action(action):
    if not robot:
        return "RobotController non initialisé. Veuillez vérifier les connexions GPIO.", 500

    state = request.args.get('state')
    print("state = : ", type(state), state)

    if action == "up":
        vitesse = session.get('Vitesse_KMH', 0)
        distance = session.get('Distance_CM', 0)
        print(f"Action: up, Vitesse: {vitesse} km/h, Distance: {distance} cm")
        if vitesse > 0 and distance > 0:
            robot.move('avant', speed_kmh=vitesse, distance_cm=distance)
    elif action == "down":
        vitesse = session.get('Vitesse_KMH', 0)
        distance = session.get('Distance_CM', 0)
        print(f"Action: down, Vitesse: {vitesse} km/h, Distance: {distance} cm")
        if vitesse > 0 and distance > 0:
            robot.move('arriere', speed_kmh=vitesse, distance_cm=distance)
    elif action == "left":
        vitesse = session.get('Vitesse_KMH', 0)
        print(f"Action: left, Vitesse: {vitesse} km/h")
        robot.turn('gauche', speed_kmh=vitesse)
    elif action == "right":
        vitesse = session.get('Vitesse_KMH', 0)
        print(f"Action: right, Vitesse: {vitesse} km/h")
        robot.turn('droite', speed_kmh=vitesse)
    elif action == "stop":
        print("Action: stop")
        robot.stop()
    
    return f"Action: {action}, State: {state}"

@app.route('/set_kmh', methods=['GET'])
def set_kmh():
    print(session)
    if session['Demarrage'] == 0:
        value = session['Vitesse_KMH']
        session['Demarrage'] = 1
    else:    
        value = float(request.args.get('value'))
        session['Vitesse_KMH'] = value
        print('valeur', request.args.get('value'))
    
    print("Vitesse (KM/H) = : ", type(session['Vitesse_KMH']), session['Vitesse_KMH'])
    D_moteur = traitement_F_M(session['Marche'])
    session['Tourne_Moteur'] = D_moteur
    print("forward------------------")
    print("avance_moteur", D_moteur, session['Vitesse_KMH'])
    print(f'Vitesse KM/H reçue : {value}')
    return jsonify({'status': 'success', 'value': value})

@app.route('/set_distance', methods=['GET'])
def set_distance():
    """Route pour définir la distance en cm."""
    value = float(request.args.get('value'))
    session['Distance_CM'] = value
    print(f'Distance en CM reçue : {value}')
    return jsonify({'status': 'success', 'value': value})

def traitement_F_M(t_f_m: float):
    # Convertit la vitesse en km/h en une sortie moteur (entre 0 et 1)
    # basée sur la vitesse maximale définie.
    résultat = t_f_m * (session['Vitesse_KMH'] / MAX_KMH)
    print(résultat, t_f_m)
    return résultat

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
