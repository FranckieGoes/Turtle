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
    WHEELBASE_CM = 52.0  # Empattement du robot en cm (distance entre les roues)
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
    
    def _run_motor_ramp_up(self, motor, target_delay, direction):
        """
        Fonction cible pour le thread de démarrage progressif d'un moteur.
        """
        motor.ramp_up(target_delay, direction, self.stop_event, self.pause_event)

    def _run_motor_ramp_down(self, motor, direction):
        """
        Fonction cible pour le thread d'arrêt progressif d'un moteur.
        """
        motor.ramp_down(direction, self.stop_event)
    
    def start_gradually(self, direction, speed_kmh, angle_deg):
        """
        Démarre le robot en ligne droite ou avec un braquage, de manière progressive.
        """
        print(f"Démarre progressivement : direction={direction}, vitesse={speed_kmh} KM/H, angle={angle_deg}°")

        self.stop() # Arrête les mouvements précédents avant d'en démarrer un nouveau
        self.stop_event.clear()
        self.pause_event.clear()

        angle_ratio = abs(angle_deg) / 10.0
        
        if angle_deg > 0: # Virage à droite
            speed_gauche = speed_kmh
            speed_droit = speed_kmh * (1 - angle_ratio)
        elif angle_deg < 0: # Virage à gauche
            speed_gauche = speed_kmh * (1 - angle_ratio)
            speed_droit = speed_kmh
        else: # Ligne droite
            speed_gauche = speed_kmh
            speed_droit = speed_kmh

        speed_gauche = max(0, speed_gauche)
        speed_droit = max(0, speed_droit)

        delay_gauche = self._calculate_delay(speed_gauche)
        delay_droit = self._calculate_delay(speed_droit)
        
        dir_gauche, dir_droit = self._get_direction_booleans(direction)

        thread_gauche = threading.Thread(target=self._run_motor_ramp_up, args=(self.motor_gauche, delay_gauche, dir_gauche))
        thread_droit = threading.Thread(target=self._run_motor_ramp_up, args=(self.motor_droit, delay_droit, dir_droit))

        self.motor_threads = [thread_gauche, thread_droit]
        
        thread_gauche.start()
        thread_droit.start()

    def stop_gradually(self):
        """
        Arrête le robot de manière progressive en ralentissant d'abord.
        """
        if self.motor_threads:
            print("Arrêt progressif des moteurs...")
            # On utilise les threads d'arrêt progressif
            thread_gauche = threading.Thread(target=self._run_motor_ramp_down, args=(self.motor_gauche, self.motor_gauche.current_direction))
            thread_droit = threading.Thread(target=self._run_motor_ramp_down, args=(self.motor_droit, self.motor_droit.current_direction))
            
            thread_gauche.start()
            thread_droit.start()

            for thread in self.motor_threads:
                if thread.is_alive():
                    thread.join()
            self.motor_threads = []
        else:
            print("Aucun mouvement en cours à arrêter.")

    # Les méthodes 'move' et 'stop' sont mises à jour pour utiliser les nouvelles méthodes progressives.
    def move(self, direction, speed_kmh, angle_deg):
        self.start_gradually(direction, speed_kmh, angle_deg)

    def stop(self):
        self.stop_gradually()

    def turn(self, direction, speed_kmh, angle_deg):
        """
        Effectue une rotation sur place.
        
        Args:
            direction (str): 'gauche' ou 'droite'.
            speed_kmh (float): Vitesse de rotation en km/h.
            angle_deg (float): Angle de rotation en degrés.
        """
        print(f"Rotation sur place : direction={direction}, vitesse={speed_kmh} KM/H, angle={angle_deg}°")

        self.stop()
        self.stop_event.clear()

        dir_gauche, dir_droit = self._get_direction_booleans(direction)
        
        distance_cm = (self.WHEELBASE_CM * 3.14159) * (angle_deg / 360.0)
        steps_to_move = int(distance_cm * self.STEPS_PER_CM)

        delay = self._calculate_delay(speed_kmh)

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
        
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)
        
    def pause(self):
        """Met le mouvement du robot en pause."""
        print("Pause du robot...")
        self.pause_event.set()
        
    def resume(self):
        """Reprend le mouvement du robot après une pause."""
        print("Reprise du robot...")
        self.pause_event.clear()
    
    def cleanup_gpio(self):
        """Nettoie toutes les broches GPIO."""
        GPIO.cleanup()