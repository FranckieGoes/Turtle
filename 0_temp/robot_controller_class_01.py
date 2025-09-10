# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import threading
import math
from stepper_motor_class import StepperMotor
from SpeedConverter import SpeedConverter

class RobotController:
    """
    Classe pour contrôler le mouvement d'un robot avec deux moteurs pas à pas.
    
    Cette classe gère les mouvements du robot (avancer, reculer, tourner, s'arrêter)
    en synchronisant deux moteurs pas à pas. Elle utilise des threads pour permettre
    des mouvements continus et des événements pour les contrôler.
    """
    
    # Constantes physiques du robot
    WHEELBASE_CM = 52.0  # Empattement du robot en cm (distance entre les roues).
                         # Cette valeur doit être calibrée précisément pour des virages précis.
    STEPS_PER_CM = 21    # Nombre de pas requis pour déplacer le robot d'un centimètre.
                         # Cette valeur dépend de la taille des roues et de la configuration du micro-pas.
    
    # Constantes pour la vitesse de base et la vitesse maximale
    MAX_SPEED_KMH = 10.0 # Vitesse maximale en km/h
    
    # Initialisation de la classe avec les broches des moteurs
    def __init__(self, motor_gauche_pins, motor_droit_pins):
        """
        Initialise les deux moteurs du robot et configure les threads de contrôle.

        Args:
            motor_gauche_pins (dict): Dictionnaire des broches pour le moteur gauche (dir_pin, step_pin, en_pin).
            motor_droit_pins (dict): Dictionnaire des broches pour le moteur droit (dir_pin, step_pin, en_pin).
        """
        print("Initialisation du RobotController...")
        self.motor_gauche = StepperMotor(**motor_gauche_pins)
        self.motor_droit = StepperMotor(**motor_droit_pins)
        
        # Événements de contrôle pour les threads
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        
        # Liste pour conserver les threads de mouvement
        self.motor_threads = []
        
        # Configuration de la classe SpeedConverter
        # Nous supposons un moteur avec 400 pas/rév (mode full step) et une roue de 61mm
        self.speed_converter = SpeedConverter(steps_per_rev=400, wheel_diameter_mm=61.0)
        
        # Assurez-vous que les moteurs sont désactivés au démarrage
        self.motor_gauche.disable()
        self.motor_droit.disable()
        print("RobotController initialisé. Prêt à recevoir des commandes.")

    def _calculate_delay(self, speed_kmh):
        """
        Calcule le délai entre les pas du moteur en fonction d'une vitesse en km/h.
        
        Args:
            speed_kmh (float): Vitesse souhaitée en kilomètres par heure.
            
        Returns:
            float: Le délai en secondes entre chaque pas.
        """
        # Gérer le cas où la vitesse est nulle ou négative
        if speed_kmh <= 0:
            return float('inf')  # Retourne l'infini pour un mouvement à l'arrêt

        # Convertir la vitesse en pas par seconde
        steps_per_sec = self.speed_converter.convert_kmh_to_steps_per_sec(speed_kmh)
        
        # S'assurer que steps_per_sec n'est pas nul pour éviter une division par zéro
        if steps_per_sec == 0:
            return float('inf')
            
        # Le délai est l'inverse du nombre de pas par seconde
        delay = 1.0 / steps_per_sec
        return delay

    def _get_direction_booleans(self, angle_deg):
        """
        Détermine la direction de chaque moteur pour effectuer une rotation.
        
        Args:
            angle_deg (int): L'angle de rotation. Positif pour un virage à droite, négatif pour un virage à gauche.
        
        Returns:
            tuple: (direction_gauche, direction_droite)
        """
        # Vraie pour la direction avant/droite, Faux pour la direction arrière/gauche
        if angle_deg >= 0:
            # Virage à droite: moteur droit vers l'arrière, moteur gauche vers l'avant
            return True, False
        else:
            # Virage à gauche: moteur droit vers l'avant, moteur gauche vers l'arrière
            return False, True

    def forward(self, speed_kmh):
        """
        Fait avancer le robot en mode continu.
        
        Args:
            speed_kmh (float): Vitesse en kilomètres par heure.
        """
        print(f"Action: Avancer à {speed_kmh} KM/H")
        
        # Arrêter tout mouvement précédent
        self.stop()
        self.stop_event.clear()
        
        # Lancer les threads pour les mouvements continus
        delay = self._calculate_delay(speed_kmh)
        self.motor_threads = [
            threading.Thread(target=self.motor_gauche.move_continuously, args=(True, self.stop_event, self.pause_event)),
            threading.Thread(target=self.motor_droit.move_continuously, args=(True, self.stop_event, self.pause_event))
        ]
        
        for thread in self.motor_threads:
            thread.start()

    def backward(self, speed_kmh):
        """
        Fait reculer le robot en mode continu.

        Args:
            speed_kmh (float): Vitesse en kilomètres par heure.
        """
        print(f"Action: Reculer à {speed_kmh} KM/H")

        # Arrêter tout mouvement précédent
        self.stop()
        self.stop_event.clear()

        # Lancer les threads pour les mouvements continus
        delay = self._calculate_delay(speed_kmh)
        self.motor_threads = [
            threading.Thread(target=self.motor_gauche.move_continuously, args=(False, self.stop_event, self.pause_event)),
            threading.Thread(target=self.motor_droit.move_continuously, args=(False, self.stop_event, self.pause_event))
        ]
        
        for thread in self.motor_threads:
            thread.start()

    def turn_on_spot(self, speed_kmh, angle_deg):
        """
        Fait pivoter le robot sur place d'un angle donné.

        Args:
            speed_kmh (float): Vitesse de rotation en km/h.
            angle_deg (int): Angle de rotation en degrés.
                             Positif pour droite, négatif pour gauche.
        """
        print(f"Rotation sur place : vitesse={speed_kmh} KM/H, angle={angle_deg}°")

        # Arrêter tout mouvement précédent
        self.stop()
        self.stop_event.clear()

        # Déterminer les directions
        dir_gauche, dir_droit = self._get_direction_booleans(angle_deg)
        
        # Calculer la distance de déplacement pour chaque roue
        # Le périmètre d'un cercle avec le diamètre de l'empattement
        circumference = self.WHEELBASE_CM * math.pi
        distance_cm = (circumference * abs(angle_deg)) / 360.0
        steps_to_move = int(distance_cm * self.STEPS_PER_CM)

        # Calculer le délai
        delay = self._calculate_delay(speed_kmh)

        # Mettre en mouvement les moteurs sur place
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
        
        # Les threads de mouvement lisent dynamiquement la vitesse.
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
        print("Moteurs arrêtés.")

    def pause(self):
        """Met le mouvement du robot en pause."""
        print("Pause du robot...")
        self.pause_event.set()

    def resume(self):
        """Reprend le mouvement du robot."""
        print("Reprise du mouvement...")
        self.pause_event.clear()

    # --- Gestion de l'enregistrement de la zone ---
    # Ces méthodes sont des stubs, prêtes à être complétées avec la logique
    # d'enregistrement et de stockage du chemin.
    def start_recording(self):
        """Démarre l'enregistrement du parcours du robot."""
        print("Démarrage de l'enregistrement du parcours.")
        # Ajoutez ici la logique pour sauvegarder les commandes de mouvement

    def stop_recording(self):
        """Arrête l'enregistrement et sauve le parcours."""
        print("Arrêt de l'enregistrement du parcours.")
        # Ajoutez ici la logique pour finaliser et sauvegarder le parcours
        
    def replay_route(self):
        """Rejoue le dernier parcours enregistré."""
        print("Relecture du parcours enregistré...")
        # Ajoutez ici la logique pour lire le chemin et le rejouer
        
    def cleanup(self):
        """Nettoie les broches GPIO pour une utilisation ultérieure."""
        print("Nettoyage des broches GPIO.")
        self.motor_gauche.cleanup()
        self.motor_droit.cleanup()
