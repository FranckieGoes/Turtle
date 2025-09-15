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
    MIN_SPEED_KMH = 0.3
    ACCEL_DURATION_S = 0.5
    RAMP_STEPS = 20
    
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
        self.ramp_thread = None
        self.current_speed_kmh = 0.0
        
        self.speed_converter = SpeedConverter(steps_per_rev=400, wheel_diameter_mm=61.0)
        
        self.motor_gauche.disable()
        self.motor_droit.disable()
        print("RobotController initialisé. Prêt à recevoir des commandes.")

    def _calculate_delay(self, speed_kmh):
        if speed_kmh <= 0: return float('inf')
        steps_per_sec = self.speed_converter.convert_kmh_to_steps_per_sec(speed_kmh)
        if steps_per_sec == 0: return float('inf')
        return 1.0 / (2 * steps_per_sec)

    def _start_thread(self, target_func, args_tuple):
        self.stop() 
        self.stop_event.clear()
        self.pause_event.clear()
        self.current_thread = threading.Thread(target=target_func, args=args_tuple)
        self.current_thread.start()
        
    def _ramp_speed(self, start_speed, end_speed, duration):
        delta_speed = end_speed - start_speed
        sleep_interval = duration / self.RAMP_STEPS
        for i in range(self.RAMP_STEPS + 1):
            step_speed = start_speed + (delta_speed * i / self.RAMP_STEPS)
            self.update_speed(step_speed)
            time.sleep(sleep_interval)

    def _start_movement_with_ramp(self, speed_kmh, direction_gauche, direction_droit):
        self.stop()
        self.update_speed(self.MIN_SPEED_KMH)
        self._start_thread(target_func=self._move_both_motors, args_tuple=(direction_gauche, direction_droit))
        self.ramp_thread = threading.Thread(target=self._ramp_speed, args=(self.MIN_SPEED_KMH, speed_kmh, self.ACCEL_DURATION_S))
        self.ramp_thread.start()

    def move_forward(self, speed_kmh):
        print(f"Déplacement avant demandé à {speed_kmh} km/h.")
        self._start_movement_with_ramp(speed_kmh, 'forward', 'backward')

    def move_backward(self, speed_kmh):
        print(f"Déplacement arrière demandé à {speed_kmh} km/h.")
        self._start_movement_with_ramp(speed_kmh, 'backward', 'forward')
    
    def select_type_rotate(self, angle_IHM, diametre_IHM, direction_IHM, vitesse_IHM):
        if diametre_IHM < self.WHEELBASE_CM:
            angle_deg = -angle_IHM if direction_IHM == "left" else angle_IHM
            self.turn_on_spot_right(vitesse_IHM, angle_deg)
        else:
            angle_deg = -angle_IHM if direction_IHM == "left" else angle_IHM
            self.make_turn(vitesse_IHM, diametre_IHM, angle_deg)  

    def turn_on_spot_right(self, speed_kmh, angle_deg):
        print(f"Rotation droite demandée de {angle_deg} degrés.")
        self._start_movement_with_ramp(speed_kmh, 'forward', 'forward')

    def turn_on_spot_left(self, speed_kmh, angle_deg):
        print(f"Rotation gauche demandée de {angle_deg} degrés.")
        self._start_movement_with_ramp(speed_kmh, 'backward', 'backward')
    
    def make_turn(self, speed_kmh, diameter_cm, angle_deg):
        print(f"Virage demandé: {angle_deg}° sur un diamètre de {diameter_cm} cm à {speed_kmh} km/h.")
        if diameter_cm < self.WHEELBASE_CM:
            print(f"Erreur: Diamètre ({diameter_cm} cm) inférieur à l'empattement ({self.WHEELBASE_CM} cm).")
            return

        turn_radius_cm = diameter_cm / 2.0
        radius_outer = turn_radius_cm + (self.WHEELBASE_CM / 2.0)
        radius_inner = turn_radius_cm - (self.WHEELBASE_CM / 2.0)
        speed_ratio = radius_inner / radius_outer
        speed_outer_kmh = speed_kmh
        speed_inner_kmh = speed_kmh * speed_ratio

        angle_rad = math.radians(abs(angle_deg))
        distance_outer_cm = radius_outer * angle_rad
        speed_outer_cm_s = (speed_outer_kmh * 100000) / 3600
        if speed_outer_cm_s == 0: return
        turn_duration_s = distance_outer_cm / speed_outer_cm_s

        if angle_deg > 0:
            delay_gauche, delay_droit = self._calculate_delay(speed_outer_kmh), self._calculate_delay(speed_inner_kmh)
            dir_gauche, dir_droit = 'forward', 'backward'
        else:
            delay_gauche, delay_droit = self._calculate_delay(speed_inner_kmh), self._calculate_delay(speed_outer_kmh)
            dir_gauche, dir_droit = 'forward', 'backward'
            
        def _execute_turn():
            self.motor_gauche.set_speed(delay_gauche)
            self.motor_droit.set_speed(delay_droit)
            move_thread = threading.Thread(target=self._move_both_motors, args=(dir_gauche, dir_droit))
            move_thread.start()
            time.sleep(turn_duration_s)
            self.stop_event.set()
            move_thread.join()
            print(f"Virage de {angle_deg}° terminé.")

        self.stop()
        self.stop_event.clear()
        self.current_thread = threading.Thread(target=_execute_turn)
        self.current_thread.start()
            
    def _move_both_motors(self, direction_gauche, direction_droit):
        thread_gauche = threading.Thread(target=self.motor_gauche.move_continuously, args=(direction_gauche, self.stop_event, self.pause_event))
        thread_droit = threading.Thread(target=self.motor_droit.move_continuously, args=(direction_droit, self.stop_event, self.pause_event))
        thread_gauche.start()
        thread_droit.start()
        thread_gauche.join()
        thread_droit.join()

    def update_speed(self, speed_kmh):
        safe_speed = max(0, min(speed_kmh, self.MAX_SPEED_KMH))
        self.current_speed_kmh = safe_speed
        delay = self._calculate_delay(safe_speed)
        self.motor_gauche.set_speed(delay)
        self.motor_droit.set_speed(delay)

    def stop(self):
        if self.current_thread and self.current_thread.is_alive():
            start_decel_speed = self.current_speed_kmh
            if start_decel_speed > self.MIN_SPEED_KMH:
                 self._ramp_speed(start_decel_speed, self.MIN_SPEED_KMH, self.ACCEL_DURATION_S)
            self.stop_event.set()
            self.current_thread.join(timeout=1.0)
        self.motor_gauche.stop_moving()
        self.motor_droit.stop_moving()
        self.current_speed_kmh = 0.0

    def cleanup(self):
        print("Nettoyage des broches GPIO.")
        self.stop()
        if self.pi: self.pi.stop()

    # --- NOUVELLES FONCTIONS POUR LE MODE AUTONOME ---

    def move_for_distance(self, speed_kmh, distance_cm):
        """
        Avance en ligne droite sur une distance précise. Mouvement bloquant.
        """
        print(f"Déplacement demandé de {distance_cm} cm à {speed_kmh} km/h.")
        speed_cm_s = (speed_kmh * 100000) / 3600
        if speed_cm_s <= 0: return
        duration_s = distance_cm / speed_cm_s

        def _execute_move():
            self._start_movement_with_ramp(speed_kmh, 'forward', 'backward')
            time.sleep(duration_s)
            self.stop()
            print("Déplacement terminé.")

        # Exécute le mouvement de manière bloquante
        move_thread = threading.Thread(target=_execute_move)
        move_thread.start()
        move_thread.join()

    def turn_on_spot_for_angle(self, speed_kmh, angle_deg):
        """
        Pivote sur place d'un angle précis. Mouvement bloquant.
        Angle positif pour la droite, négatif pour la gauche.
        """
        print(f"Rotation sur place demandée de {angle_deg} degrés.")
        
        # Le chemin parcouru par une roue est un arc de cercle dont le rayon est la moitié de l'empattement
        distance_cm = (math.pi * self.WHEELBASE_CM) * (abs(angle_deg) / 360.0)
        speed_cm_s = (speed_kmh * 100000) / 3600
        if speed_cm_s <= 0: return
        duration_s = distance_cm / speed_cm_s

        if angle_deg > 0: # Droite
            dirs = ('forward', 'forward')
        else: # Gauche
            dirs = ('backward', 'backward')

        def _execute_turn():
            self._start_movement_with_ramp(speed_kmh, dirs[0], dirs[1])
            time.sleep(duration_s)
            self.stop()
            print("Rotation terminée.")
        
        # Exécute la rotation de manière bloquante
        turn_thread = threading.Thread(target=_execute_turn)
        turn_thread.start()
        turn_thread.join()