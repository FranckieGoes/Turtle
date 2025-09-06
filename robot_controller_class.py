# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import threading
from stepper_motor_class import StepperMotor
from SpeedConverter import SpeedConverter

class RobotController:
    """
    Classe pour contrôler le mouvement d'un robot avec deux moteurs pas à pas.
    """
    # Constantes physiques du robot
    WHEELBASE_CM = 15.0  # Empattement du robot en cm (distance entre les roues)
    STEPS_PER_CM = 21    # Nombre de pas requis pour déplacer le robot d'un centimètre.
                         # C'est une valeur à calibrer en fonction de la taille des roues et de la configuration du micro-pas.
    
    # Constantes pour la vitesse de base et la vitesse maximale
    MAX_SPEED_KMH = 10.0 # Vitesse maximale en km/h
    
    # Initialisation de la classe avec les broches des moteurs
    def __init__(self, motor_gauche_pins, motor_droit_pins, steps_per_cm=21):
        """
        Initialise les deux moteurs du robot.

        Args:
            motor_gauche_pins (dict): Dictionnaire des broches pour le moteur gauche (dir_pin, step_pin, en_pin).
            motor_droit_pins (dict): Dictionnaire des broches pour le moteur droit (dir_pin, step_pin, en_pin).
            steps_per_cm (int): Nombre de pas requis pour déplacer le robot d'un centimètre.
        """
        # S'assurer que les deux moteurs partagent la même broche d'activation pour un contrôle synchronisé
        if motor_gauche_pins['en_pin'] != motor_droit_pins['en_pin']:
            print("AVERTISSEMENT: Les moteurs devraient partager une seule broche d'activation (en_pin).")

        self.STEPS_PER_CM = steps_per_cm
        
        # Création des instances des moteurs pas à pas
        self.motor_gauche = StepperMotor(
            dir_pin=motor_gauche_pins['dir_pin'],
            step_pin=motor_gauche_pins['step_pin'],
            en_pin=motor_gauche_pins['en_pin']
        )
        self.motor_droit = StepperMotor(
            dir_pin=motor_droit_pins['dir_pin'],
            step_pin=motor_droit_pins['step_pin'],
            en_pin=motor_droit_pins['en_pin']
        )

        # Création d'un convertisseur de vitesse (à adapter aux spécifications des roues et des pas)
        # Par défaut, 400 pas par tour et un diamètre de roue de 61 mm
        self.speed_converter = SpeedConverter(
            steps_per_rev=400,
            wheel_diameter_mm=61.0
        )
        
        # Événements de contrôle pour les threads
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        self.motor_threads = []

    def _calculate_delay(self, speed_kmh):
        """
        Calcule le délai entre les pas en fonction de la vitesse en km/h.
        
        Args:
            speed_kmh (float): Vitesse en kilomètres par heure (km/h).
            
        Returns:
            float: Le délai en secondes entre les pas.
        """
        # Utilise la classe SpeedConverter pour obtenir les pas par seconde
        steps_per_sec = self.speed_converter.convert_kmh_to_steps_per_sec(speed_kmh)
        
        if steps_per_sec > 0:
            delay = 1.0 / (2 * steps_per_sec)
        else:
            delay = 0  # Délai infini pour l'arrêt
            
        return delay

    def _get_direction_booleans(self, direction):
        """
        Convertit une direction textuelle en booléens pour les broches DIR des moteurs.
        'avant' -> (True, True)
        'arriere' -> (False, False)
        'gauche' (rotation) -> (False, True)
        'droite' (rotation) -> (True, False)
        """
        if direction == 'avant':
            return (True, True)
        elif direction == 'arriere':
            return (False, False)
        elif direction == 'gauche':
            return (False, True)
        elif direction == 'droite':
            return (True, False)
        else:
            return (False, False)

    def _run_motor(self, motor, steps_to_move, direction, delay_sec):
        """
        Fonction cible pour le thread de contrôle d'un moteur.
        Gère la pause et l'arrêt.
        """
        motor.enable()
        motor.set_direction(direction)
        
        for i in range(steps_to_move):
            # Vérifie si le robot doit s'arrêter
            if self.stop_event.is_set():
                break
            
            # Attendre si le robot est en pause
            while self.pause_event.is_set():
                time.sleep(0.1)
                if self.stop_event.is_set():
                    break
            
            # Exécution du pas
            motor.step(delay_sec)

        motor.disable()
        print(f"Moteur {motor.STEP_PIN} a terminé ses pas.")

    def move(self, direction, speed_kmh, angle_deg):
        """
        Déplace le robot en ligne droite ou avec un rayon de braquage.

        Args:
            direction (str): 'avant' ou 'arriere'.
            speed_kmh (float): Vitesse en km/h.
            angle_deg (float): Angle de braquage en degrés. Positif pour la droite, négatif pour la gauche.
                                Si 0, le robot se déplace en ligne droite.
        """
        print(f"Déplacement : direction={direction}, vitesse={speed_kmh} KM/H, angle={angle_deg}°")
        
        self.stop() # Arrête les mouvements précédents avant d'en démarrer un nouveau
        self.stop_event.clear()
        self.pause_event.clear()
        
        # Calcul des vitesses individuelles des roues en fonction de l'angle
        # Ceci est une approximation simplifiée
        angle_ratio = abs(angle_deg) / 10.0 # Normaliser l'angle (max 10°)
        
        if angle_deg > 0: # Virage à droite
            speed_gauche = speed_kmh
            speed_droit = speed_kmh * (1 - angle_ratio)
        elif angle_deg < 0: # Virage à gauche
            speed_gauche = speed_kmh * (1 - angle_ratio)
            speed_droit = speed_kmh
        else: # Ligne droite
            speed_gauche = speed_kmh
            speed_droit = speed_kmh

        # S'assurer que les vitesses ne sont pas négatives
        speed_gauche = max(0, speed_gauche)
        speed_droit = max(0, speed_droit)

        delay_gauche = self._calculate_delay(speed_gauche)
        delay_droit = self._calculate_delay(speed_droit)

        # Définition des directions de déplacement
        dir_gauche, dir_droit = self._get_direction_booleans(direction)

        # Les mouvements des roues sont continus jusqu'à l'envoi d'un nouvel ordre.
        # On définit un nombre de pas arbitrairement grand pour simuler un mouvement continu.
        steps_to_move_gauche = 1000000 
        steps_to_move_droit = 1000000

        # Création et lancement des threads pour les moteurs
        thread_gauche = threading.Thread(target=self._run_motor, args=(self.motor_gauche, steps_to_move_gauche, dir_gauche, delay_gauche))
        thread_droit = threading.Thread(target=self._run_motor, args=(self.motor_droit, steps_to_move_droit, dir_droit, delay_droit))

        self.motor_threads = [thread_gauche, thread_droit]
        
        thread_gauche.start()
        thread_droit.start()

    def turn(self, direction, speed_kmh, angle_deg):
        """
        Effectue une rotation sur place.
        
        Args:
            direction (str): 'gauche' ou 'droite'.
            speed_kmh (float): Vitesse de rotation en km/h.
            angle_deg (float): Angle de rotation en degrés.
        """
        print(f"Rotation sur place : direction={direction}, vitesse={speed_kmh} KM/H, angle={angle_deg}°")

        self.stop() # Arrête les mouvements précédents avant d'en démarrer un nouveau
        self.stop_event.clear()

        # Pour une rotation sur place, les moteurs tournent dans des directions opposées
        dir_gauche, dir_droit = self._get_direction_booleans(direction)
        
        # Calcul du nombre de pas pour la rotation
        # La distance parcourue par les roues est la circonférence du cercle de braquage (2 * pi * rayon)
        # où le rayon est la moitié de l'empattement.
        # On utilise une formule simplifiée pour un virage sur place
        distance_cm = (self.WHEELBASE_CM * 3.14159) * (angle_deg / 360.0)
        steps_to_move = int(distance_cm * self.STEPS_PER_CM)

        delay = self._calculate_delay(speed_kmh)

        # Utilisation de la méthode move_steps de la classe StepperMotor sans threading
        # car c'est un mouvement court et bloquant
        self.motor_gauche.set_direction(dir_gauche)
        self.motor_gauche.move_steps(steps_to_move, delay)
        
        self.motor_droit.set_direction(dir_droit)
        self.motor_droit.move_steps(steps_to_move, delay)

    def update_speed(self, speed_kmh):
        """
        Met à jour la vitesse des moteurs en cours de route.
        """
        delay = self._calculate_delay(speed_kmh)
        print(f"Vitesse mise à jour en dynamique. Nouveau délai: {delay} sec")
        
        # Les threads de mouvement lisent dynamiquement la vitesse
        # Il suffit de mettre à jour le délai dans chaque moteur.
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)

    def stop(self):
        """Arrête les deux moteurs et les désactive."""
        if self.motor_threads:
            print("Arrêt des moteurs...")
            self.stop_event.set()
            for thread in self.motor_threads:
                if thread.is_alive():
                    thread.join()
            self.motor_threads = []
            self.motor_gauche.disable()
            self.motor_droit.disable()
        else:
            print("Aucun mouvement en cours à arrêter.")

    def pause(self):
        """Met le mouvement du robot en pause."""
        print("Pause du robot...")
        self.pause_event.set()
        
    def resume(self):
        """Reprend le mouvement du robot après une pause."""
        print("Reprise du robot...")
        self.pause_event.clear()
        
