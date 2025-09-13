# -*- coding: utf-8 -*-
import pigpio
import time
import threading
import math
import sys
from stepper_motor_class import StepperMotor
from SpeedConverter import SpeedConverter

class RobotController:
    """
    Classe pour contrôler le mouvement d'un robot avec deux moteurs pas à pas,
    en utilisant la bibliothèque pigpio pour un contrôle précis des GPIO.
    """
    
    WHEELBASE_CM = 52.0
    STEPS_PER_CM = 21
    MAX_SPEED_KMH = 10.0
    
    def __init__(self, pi, motor_gauche_pins, motor_droit_pins):
        print("Initialisation du RobotController avec pigpio...")
        
        self.pi = pi
        if not self.pi.connected:
            print("Erreur : le démon pigpiod n'est pas en cours d'exécution ou la connexion a échoué.")
            sys.exit(1)
        
        self.motor_gauche = StepperMotor(pi=self.pi, **motor_gauche_pins)
        self.motor_droit = StepperMotor(pi=self.pi, **motor_droit_pins)
        
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        
        self.current_thread = None
        
        self.speed_converter = SpeedConverter(steps_per_rev=400, wheel_diameter_mm=61.0)
        
        self.motor_gauche.disable()
        self.motor_droit.disable()
        print("RobotController initialisé. Prêt à recevoir des commandes.")

    def _calculate_delay(self, speed_kmh):
        if speed_kmh <= 0:
            return float('inf')
        steps_per_sec = self.speed_converter.convert_kmh_to_steps_per_sec(speed_kmh)
        if steps_per_sec == 0:
            return float('inf')
        return 1.0 / (2 * steps_per_sec)

    def _start_thread(self, target_func, args_tuple):
        """Démarre un nouveau thread de mouvement, en s'assurant que le précédent est arrêté."""
        self.stop() # Arrête tout mouvement en cours avant d'en commencer un nouveau
        self.stop_event.clear()
        self.pause_event.clear()
        
        self.current_thread = threading.Thread(target=target_func, args=args_tuple)
        self.current_thread.start()

    def move_forward(self, speed_kmh):
        """Déplace le robot vers l'avant."""
        print(f"Déplacement vers l'avant à {speed_kmh} km/h.")
        self.update_speed(speed_kmh)
        self._start_thread(target_func=self._move_both_motors, args_tuple=('forward', 'backward'))

    def move_backward(self, speed_kmh):
        """Déplace le robot vers l'arrière."""
        print(f"Déplacement vers l'arrière à {speed_kmh} km/h.")
        self.update_speed(speed_kmh)
        self._start_thread(target_func=self._move_both_motors, args_tuple=('backward', 'forward'))

    def turn_on_spot_right(self, speed_kmh, angle_deg):
        """Fait pivoter le robot sur place à droite."""
        print(f"Rotation sur place de {angle_deg} degrés.")
        self.update_speed(speed_kmh)
        self._start_thread(target_func=self._move_both_motors, args_tuple=('forward', 'forward'))

    def turn_on_spot_left(self, speed_kmh, angle_deg):
        """Fait pivoter le robot sur place à gauche."""
        print(f"Rotation sur place de {angle_deg} degrés.")
        self.update_speed(speed_kmh)
        self._start_thread(target_func=self._move_both_motors, args_tuple=('backward', 'backward'))
            
    def _move_both_motors(self, direction_gauche, direction_droit):
        """Fonction cible pour les threads de mouvement."""
        # Crée des threads séparés pour chaque moteur
        thread_gauche = threading.Thread(target=self.motor_gauche.move_continuously, args=(direction_gauche, self.stop_event, self.pause_event))
        thread_droit = threading.Thread(target=self.motor_droit.move_continuously, args=(direction_droit, self.stop_event, self.pause_event))
        
        thread_gauche.start()
        thread_droit.start()
        
        thread_gauche.join()
        thread_droit.join()

    def update_speed(self, speed_kmh):
        safe_speed = max(0, min(speed_kmh, self.MAX_SPEED_KMH))
        delay = self._calculate_delay(safe_speed)
        print(f"Vitesse mise à jour. Vitesse: {safe_speed} KM/H, Nouveau délai: {delay} sec")
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)

    def stop(self):
        """Arrête tout mouvement en cours."""
        if self.current_thread and self.current_thread.is_alive():
            print("Arrêt du thread de mouvement...")
            self.stop_event.set()
            self.current_thread.join(timeout=1.0) # Attendre que le thread se termine
        
        self.motor_gauche.stop_moving()
        self.motor_droit.stop_moving()
        print("Moteurs arrêtés.")

    def pause(self):
        print("Pause du robot...")
        self.pause_event.set()

    def resume(self):
        print("Reprise du mouvement...")
        self.pause_event.clear()
        
    def cleanup(self):
        print("Nettoyage des broches GPIO.")
        self.stop()
        if self.pi:
            self.pi.stop()
            print("Instance pigpio arrêtée.")