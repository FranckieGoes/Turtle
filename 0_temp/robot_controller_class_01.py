# Fichier : robot_controller_class.py (MIS À JOUR)

import pigpio
import time
import threading
import math
import sys
from stepper_motor_class import StepperMotor
from SpeedConverter import SpeedConverter

class RobotController:
    WHEELBASE_CM = 52.0
    STEPS_PER_CM = 21 # Constante clé pour la conversion distance -> pas
    MAX_SPEED_KMH = 10.0
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
        return 1.0 / (2 * steps_per_sec) if steps_per_sec > 0 else float('inf')

    # --- Fonctions de mouvement continu (inchangées) ---
    def _start_thread(self, target_func, args_tuple): # ...
        self.stop(); self.stop_event.clear(); self.pause_event.clear(); self.current_thread = threading.Thread(target=target_func, args=args_tuple); self.current_thread.start()
    
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
    
    def move_forward(self, speed_kmh): self._start_movement_with_ramp(speed_kmh, 'forward', 'backward')
    
    def move_backward(self, speed_kmh): self._start_movement_with_ramp(speed_kmh, 'backward', 'forward')
    
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
    
    # --- Fonctions de sélection de virage (modifiées pour appeler les bonnes fonctions) ---
    def select_type_rotate(self, angle_IHM, diametre_IHM, direction_IHM, vitesse_IHM):
        angle_deg = -angle_IHM if direction_IHM == "left" else angle_IHM
        print("select_type_rotate")
        # Note: make_turn n'est pas encore converti au comptage de pas.
        # Seuls les virages sur place sont améliorés pour l'instant.
        if diametre_IHM < self.WHEELBASE_CM:
             self.turn_on_spot_for_angle(vitesse_IHM, angle_deg)
             print("turn_on_spot_for_angle")
        else:
             self.make_turn(vitesse_IHM, diametre_IHM, angle_deg) # Garde l'ancienne méthode pour les virages larges
             print("make_turn")

    # --- Fonctions pour le mode autonome (MISES À JOUR AVEC COMPTAGE DE PAS) ---
    def move_for_distance(self, speed_kmh, distance_cm):
        """Avance en ligne droite sur une distance précise en comptant les pas."""
        print(f"Déplacement de {distance_cm} cm demandé...")
        
        # 1. Calculer le nombre de pas
        total_steps = int(distance_cm * self.STEPS_PER_CM)
        if total_steps <= 0: return

        # 2. Calculer le délai entre les pas pour respecter la vitesse
        delay = self._calculate_delay(speed_kmh)

        # 3. Créer et lancer les threads pour chaque moteur
        thread_gauche = threading.Thread(target=self.motor_gauche.move_steps, args=(total_steps, 'forward', delay))
        thread_droit = threading.Thread(target=self.motor_droit.move_steps, args=(total_steps, 'backward', delay))
        
        thread_gauche.start()
        thread_droit.start()
        
        # 4. Attendre la fin des deux mouvements (rend la fonction bloquante)
        thread_gauche.join()
        thread_droit.join()
        
        print("Déplacement terminé.")

    def turn_on_spot_for_angle(self, speed_kmh, angle_deg):
        """Pivote sur place d'un angle précis en comptant les pas."""
        print(f"Rotation de {angle_deg} degrés demandée...")
        
        # 1. Calculer la distance de l'arc de cercle pour une roue
        arc_distance_cm = (math.pi * self.WHEELBASE_CM) * (abs(angle_deg) / 360.0)
        total_steps = int(arc_distance_cm * self.STEPS_PER_CM)
        if total_steps <= 0: return

        # 2. Calculer le délai
        delay = self._calculate_delay(speed_kmh)

        # 3. Déterminer la direction et lancer les threads
        dir_gauche = 'backward' if angle_deg < 0 else 'forward'
        dir_droit = 'backward' if angle_deg < 0 else 'forward'
        
        thread_gauche = threading.Thread(target=self.motor_gauche.move_steps, args=(total_steps, dir_gauche, delay))
        thread_droit = threading.Thread(target=self.motor_droit.move_steps, args=(total_steps, dir_droit, delay))
        
        thread_gauche.start()
        thread_droit.start()
        
        # 4. Attendre la fin
        thread_gauche.join()
        thread_droit.join()
        
        print("Rotation terminée.")
    
    # ... (Autres fonctions comme make_turn, cleanup, pause, resume restent inchangées)
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
    
    def cleanup(self): self.stop(); self.pi.stop()
    
    def pause(self): self.pause_event.set()
    
    def resume(self): self.pause_event.clear()