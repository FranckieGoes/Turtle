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
    """
    
    WHEELBASE_CM = 52.0
    STEPS_PER_CM = 21
    MAX_SPEED_KMH = 10.0
    
    def __init__(self, motor_gauche_pins, motor_droit_pins):
        print("Initialisation du RobotController...")
        self.motor_gauche = StepperMotor(**motor_gauche_pins)
        self.motor_droit = StepperMotor(**motor_droit_pins)
        
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()
        
        self.motor_threads = []
        
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
            
        return 1.0 / steps_per_sec

    def _get_direction_booleans(self, angle_deg):
        if angle_deg >= 0:
            return True, False
        else:
            return False, True

    def forward(self, speed_kmh):
        print(f"Action: Avancer à {speed_kmh} KM/H")
        
        self.stop()
        self.stop_event.clear()

        # FIX: Set the initial speed before starting the threads
        self.update_speed(speed_kmh)
        
        self.motor_threads = [
            threading.Thread(target=self.motor_gauche.move_continuously, args=(True, self.stop_event, self.pause_event)),
            threading.Thread(target=self.motor_droit.move_continuously, args=(True, self.stop_event, self.pause_event))
        ]
        
        for thread in self.motor_threads:
            thread.start()

    def backward(self, speed_kmh):
        print(f"Action: Reculer à {speed_kmh} KM/H")

        self.stop()
        self.stop_event.clear()

        # FIX: Set the initial speed before starting the threads
        self.update_speed(speed_kmh)

        self.motor_threads = [
            threading.Thread(target=self.motor_gauche.move_continuously, args=(False, self.stop_event, self.pause_event)),
            threading.Thread(target=self.motor_droit.move_continuously, args=(False, self.stop_event, self.pause_event))
        ]
        
        for thread in self.motor_threads:
            thread.start()

    def turn_on_spot(self, speed_kmh, angle_deg):
        print(f"Rotation sur place : vitesse={speed_kmh} KM/H, angle={angle_deg}°")

        self.stop()
        
        dir_gauche, dir_droit = self._get_direction_booleans(angle_deg)
        
        circumference = self.WHEELBASE_CM * math.pi
        distance_cm = (circumference * abs(angle_deg)) / 360.0
        steps_to_move = int(distance_cm * self.STEPS_PER_CM)

        delay = self._calculate_delay(speed_kmh)
        
        # Create threads for turning so the UI is not blocked
        # Note: move_steps is a blocking call, so we thread them.
        turn_thread_gauche = threading.Thread(target=self.motor_gauche.move_steps, args=(steps_to_move, dir_gauche, delay))
        turn_thread_droit = threading.Thread(target=self.motor_droit.move_steps, args=(steps_to_move, dir_droit, delay))

        turn_thread_gauche.start()
        turn_thread_droit.start()


    def update_speed(self, speed_kmh):
        # Clamp the speed to the maximum allowed
        safe_speed = max(0, min(speed_kmh, self.MAX_SPEED_KMH))
        delay = self._calculate_delay(safe_speed)
        print(f"Vitesse mise à jour. Vitesse: {safe_speed} KM/H, Nouveau délai: {delay} sec")
        
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)

    def stop(self):
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
        print("Pause du robot...")
        self.pause_event.set()

    def resume(self):
        print("Reprise du mouvement...")
        self.pause_event.clear()
        
    def cleanup(self):
        print("Nettoyage des broches GPIO.")
        self.stop() # Ensure motors are stopped
        GPIO.cleanup()

    # --- Methods for recording zone (stubs) ---
    def start_recording(self):
        print("Démarrage de l'enregistrement du parcours.")

    def stop_recording(self):
        print("Arrêt de l'enregistrement du parcours.")
        
    def replay_route(self):
        print("Relecture du parcours enregistré...")