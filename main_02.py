# -*- coding: utf-8 -*-
import time
from robot_controller_class import RobotController

def main():
    """
    Fonction principale de démonstration du RobotController.
    """
    # Définition des broches GPIO pour chaque moteur
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

    try:
        # Création de l'instance du contrôleur de robot
        robot = RobotController(motor_gauche_pins, motor_droit_pins)
        
        # Exemples d'utilisation
        
        # Avancer de 50 cm à 0.5 km/h
        robot.move('avant', speed_kmh=6, distance_cm=95)
        time.sleep(2)

        # Reculer de 50 cm à 0.2 km/h
        #robot.move('arriere', speed_kmh=50, distance_cm=50)
        #time.sleep(2)

        # Tourner à droite de 90 degrés avec un rayon de 200 cm
        #robot.turn('droite', speed_kmh=150, rayon_cm=100, angle_deg=90)
        #time.sleep(5)
        
    except KeyboardInterrupt:
        print("\nProgramme interrompu.")
    finally:
        robot.stop()

if __name__ == "__main__":
    main()
