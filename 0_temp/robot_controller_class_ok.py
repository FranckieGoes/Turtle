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
    Intègre des rampes d'accélération et de décélération.
    """
    
    # --- Constantes de configuration ---
    WHEELBASE_CM = 52.0
    STEPS_PER_CM = 21
    MAX_SPEED_KMH = 10.0
    
    # --- Nouvelles constantes pour les rampes de vitesse ---
    MIN_SPEED_KMH = 0.3      # Vitesse minimale pour démarrer/arrêter en douceur
    ACCEL_DURATION_S = 0.5   # Durée en secondes pour l'accélération/décélération
    RAMP_STEPS = 20          # Nombre d'étapes dans la rampe
    
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
        self.ramp_thread = None # Thread pour gérer l'accélération
        self.current_speed_kmh = 0.0
        
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
        self.stop() 
        self.stop_event.clear()
        self.pause_event.clear()
        
        self.current_thread = threading.Thread(target=target_func, args=args_tuple)
        self.current_thread.start()
        
    def _ramp_speed(self, start_speed, end_speed, duration):
        """
        Fonction de rampe de vitesse (bloquante).
        Fait varier la vitesse de `start_speed` à `end_speed` sur `duration` secondes.
        """
        delta_speed = end_speed - start_speed
        sleep_interval = duration / self.RAMP_STEPS
        
        for i in range(self.RAMP_STEPS + 1):
            step_speed = start_speed + (delta_speed * i / self.RAMP_STEPS)
            self.update_speed(step_speed)
            time.sleep(sleep_interval)

    def _start_movement_with_ramp(self, speed_kmh, direction_gauche, direction_droit):
        """
        Commence un mouvement en démarrant à vitesse minimale puis en accélérant.
        """
        # D'abord, on arrête tout mouvement précédent.
        self.stop()
        
        # On démarre les moteurs à la vitesse minimale
        self.update_speed(self.MIN_SPEED_KMH)
        self._start_thread(target_func=self._move_both_motors, args_tuple=(direction_gauche, direction_droit))
        
        # Ensuite, on lance l'accélération dans un thread séparé pour ne pas bloquer
        self.ramp_thread = threading.Thread(target=self._ramp_speed, args=(self.MIN_SPEED_KMH, speed_kmh, self.ACCEL_DURATION_S))
        self.ramp_thread.start()

    def move_forward(self, speed_kmh):
        """Déplace le robot vers l'avant avec une accélération progressive."""
        print(f"Déplacement avant demandé à {speed_kmh} km/h.")
        self._start_movement_with_ramp(speed_kmh, 'forward', 'backward')

    def move_backward(self, speed_kmh):
        """Déplace le robot vers l'arrière avec une accélération progressive."""
        print(f"Déplacement arrière demandé à {speed_kmh} km/h.")
        self._start_movement_with_ramp(speed_kmh, 'backward', 'forward')

    def turn_on_spot_right(self, speed_kmh, angle_deg):
        """Fait pivoter le robot sur place à droite avec une accélération progressive."""
        print(f"Rotation droite demandée de {angle_deg} degrés.")
        self._start_movement_with_ramp(speed_kmh, 'forward', 'forward')

    def turn_on_spot_left(self, speed_kmh, angle_deg):
        """Fait pivoter le robot sur place à gauche avec une accélération progressive."""
        print(f"Rotation gauche demandée de {angle_deg} degrés.")
        self._start_movement_with_ramp(speed_kmh, 'backward', 'backward')
            
    def _move_both_motors(self, direction_gauche, direction_droit):
        """Fonction cible pour les threads de mouvement."""
        thread_gauche = threading.Thread(target=self.motor_gauche.move_continuously, args=(direction_gauche, self.stop_event, self.pause_event))
        thread_droit = threading.Thread(target=self.motor_droit.move_continuously, args=(direction_droit, self.stop_event, self.pause_event))
        
        thread_gauche.start()
        thread_droit.start()
        
        thread_gauche.join()
        thread_droit.join()

    def update_speed(self, speed_kmh):
        """Met à jour la vitesse des moteurs et conserve la vitesse actuelle."""
        safe_speed = max(0, min(speed_kmh, self.MAX_SPEED_KMH))
        self.current_speed_kmh = safe_speed
        
        delay = self._calculate_delay(safe_speed)
        # Ne pas afficher pour les rampes pour éviter de spammer la console
        # print(f"Vitesse mise à jour. Vitesse: {safe_speed} KM/H, Nouveau délai: {delay} sec")
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)

    def stop(self):
        """Arrête tout mouvement en cours avec une décélération progressive."""
        if self.current_thread and self.current_thread.is_alive():
            print("Arrêt progressif du robot...")
            
            # S'assurer que toute accélération est terminée avant de décélérer
            if self.ramp_thread and self.ramp_thread.is_alive():
                self.ramp_thread.join()
            
            # Lancer la rampe de décélération (bloquante)
            start_decel_speed = self.current_speed_kmh
            if start_decel_speed > self.MIN_SPEED_KMH:
                 self._ramp_speed(start_decel_speed, self.MIN_SPEED_KMH, self.ACCEL_DURATION_S)
            
            # Stopper les threads moteurs
            self.stop_event.set()
            self.current_thread.join(timeout=1.0)
        
        self.motor_gauche.stop_moving()
        self.motor_droit.stop_moving()
        self.current_speed_kmh = 0.0
        # print("Moteurs arrêtés.") # Commenté pour une sortie plus propre

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