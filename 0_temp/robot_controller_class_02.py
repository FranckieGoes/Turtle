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
    ACCEL_DURATION_S = 1.0
    RAMP_STEPS = 50
    
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
    
    # FONCTION MODIFIÉE : select_type_rotate avec rampe
    def select_type_rotate(self, angle_IHM, diametre_IHM, direction_IHM, vitesse_IHM):
        angle_deg = -angle_IHM if direction_IHM == "left" else angle_IHM
        print("select_type_rotate avec rampe")
        
        if diametre_IHM < self.WHEELBASE_CM:
            # NOUVEAU : Utiliser la fonction avec rampe
            self._start_turn_with_ramp(vitesse_IHM, angle_deg)
            print("turn_on_spot avec rampe d'accélération/décélération")
        else:
            # Pour les virages larges, on peut aussi ajouter la rampe
            self._start_wide_turn_with_ramp(vitesse_IHM, diametre_IHM, angle_deg)
            print("make_turn avec rampe")



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

    # NOUVELLE FONCTION : Démarrage avec rampe pour les virages
    def _start_turn_with_ramp(self, speed_kmh, angle_deg):
        """Démarre un virage sur place avec rampe d'accélération/décélération"""
        self.stop()
        
        # Calculer les paramètres du virage
        arc_distance_cm = (math.pi * self.WHEELBASE_CM) * (abs(angle_deg) / 360.0)
        total_steps = int(arc_distance_cm * self.STEPS_PER_CM)
        if total_steps <= 0:
            return
            
        # Directions selon l'angle
        dir_gauche = 'backward' if angle_deg < 0 else 'forward'
        dir_droit = 'backward' if angle_deg < 0 else 'forward'
        
        # 1. Phase d'accélération
        self.update_speed(self.MIN_SPEED_KMH)
        self._start_thread(
            target_func=self._execute_precise_turn, 
            args_tuple=(total_steps, dir_gauche, dir_droit, speed_kmh)
        )
        
        # 2. Rampe d'accélération en parallèle
        self.ramp_thread = threading.Thread(
            target=self._ramp_speed, 
            args=(self.MIN_SPEED_KMH, speed_kmh, self.ACCEL_DURATION_S)
        )
        self.ramp_thread.start()
    
    def _execute_precise_turn(self, total_steps, dir_gauche, dir_droit, target_speed):
        """Exécute le virage précis avec comptage de pas et rampe de décélération"""
        steps_completed = 0
        
        # Calculer le nombre de pas pour commencer la décélération
        decel_start_steps = max(0, total_steps - int(target_speed * self.STEPS_PER_CM * 2))
        
        # Phase 1 : Accélération (gérée par le thread de rampe)
        time.sleep(self.ACCEL_DURATION_S + 0.1)  # Attendre la fin de l'accélération
        
        # Phase 2 : Vitesse constante + début de décélération
        while steps_completed < total_steps and not self.stop_event.is_set():
            # Commencer la décélération avant la fin
            if steps_completed >= decel_start_steps and self.current_speed_kmh > self.MIN_SPEED_KMH:
                remaining_steps = total_steps - steps_completed
                decel_progress = 1.0 - (remaining_steps / (total_steps - decel_start_steps))
                new_speed = self.MIN_SPEED_KMH + (target_speed - self.MIN_SPEED_KMH) * (1.0 - decel_progress)
                self.update_speed(max(self.MIN_SPEED_KMH, new_speed))
            
            # Faire un pas sur chaque moteur
            delay = self._calculate_delay(self.current_speed_kmh)
            
            # Synchronisation des pas
            thread_gauche = threading.Thread(
                target=self.motor_gauche.move_steps, 
                args=(1, dir_gauche, delay)
            )
            thread_droit = threading.Thread(
                target=self.motor_droit.move_steps, 
                args=(1, dir_droit, delay)
            )
            
            thread_gauche.start()
            thread_droit.start()
            thread_gauche.join()
            thread_droit.join()
            
            steps_completed += 1
            
            # Pause si nécessaire
            if self.pause_event.is_set():
                self.pause_event.wait()
        
        # Arrêt final
        self.current_speed_kmh = 0.0
        print(f"Virage terminé - {steps_completed}/{total_steps} pas effectués")

    # BONUS : Version avec rampe pour les virages larges aussi
    def _start_wide_turn_with_ramp(self, speed_kmh, diameter_cm, angle_deg):
        """Démarre un virage large avec rampe"""
        self.stop()
        
        if diameter_cm < self.WHEELBASE_CM:
            print(f"Erreur: Diamètre ({diameter_cm} cm) inférieur à l'empattement ({self.WHEELBASE_CM} cm).")
            return

        # Calculs géométriques (identiques à make_turn)
        turn_radius_cm = diameter_cm / 2.0
        radius_outer = turn_radius_cm + (self.WHEELBASE_CM / 2.0)
        radius_inner = turn_radius_cm - (self.WHEELBASE_CM / 2.0)
        speed_ratio = radius_inner / radius_outer
        
        angle_rad = math.radians(abs(angle_deg))
        distance_outer_cm = radius_outer * angle_rad
        
        # Vitesses différentielles
        if angle_deg > 0:  # Virage à droite
            speed_gauche, speed_droit = speed_kmh, speed_kmh * speed_ratio
            dir_gauche, dir_droit = 'forward', 'backward'
        else:  # Virage à gauche
            speed_gauche, speed_droit = speed_kmh * speed_ratio, speed_kmh
            dir_gauche, dir_droit = 'forward', 'backward'
        
        # Durée estimée du virage
        speed_outer_cm_s = (speed_kmh * 100000) / 3600
        turn_duration_s = distance_outer_cm / speed_outer_cm_s if speed_outer_cm_s > 0 else 0
        
        # Démarrer avec rampe
        self.update_speed(self.MIN_SPEED_KMH)
        self._start_thread(
            target_func=self._execute_wide_turn_with_ramp,
            args_tuple=(speed_gauche, speed_droit, dir_gauche, dir_droit, turn_duration_s)
        )
        
        # Rampe d'accélération
        self.ramp_thread = threading.Thread(
            target=self._ramp_speed,
            args=(self.MIN_SPEED_KMH, speed_kmh, self.ACCEL_DURATION_S)
        )
        self.ramp_thread.start()

    def _execute_wide_turn_with_ramp(self, speed_gauche, speed_droit, dir_gauche, dir_droit, total_duration):
        """Exécute un virage large avec décélération progressive"""
        start_time = time.time()
        
        # Attendre la fin de l'accélération
        time.sleep(self.ACCEL_DURATION_S + 0.1)
        
        # Calculer quand commencer la décélération
        decel_start_time = start_time + total_duration - self.ACCEL_DURATION_S
        
        # Démarrer les moteurs
        thread_gauche = threading.Thread(
            target=self.motor_gauche.move_continuously,
            args=(dir_gauche, self.stop_event, self.pause_event)
        )
        thread_droit = threading.Thread(
            target=self.motor_droit.move_continuously,
            args=(dir_droit, self.stop_event, self.pause_event)
        )
        
        thread_gauche.start()
        thread_droit.start()
        
        # Boucle de contrôle avec décélération progressive
        while time.time() < start_time + total_duration and not self.stop_event.is_set():
            current_time = time.time()
            
            # Phase de décélération
            if current_time >= decel_start_time:
                remaining_time = start_time + total_duration - current_time
                total_decel_time = self.ACCEL_DURATION_S
                decel_progress = 1.0 - (remaining_time / total_decel_time)
                
                current_speed_gauche = self.MIN_SPEED_KMH + (speed_gauche - self.MIN_SPEED_KMH) * (1.0 - decel_progress)
                current_speed_droit = self.MIN_SPEED_KMH + (speed_droit - self.MIN_SPEED_KMH) * (1.0 - decel_progress)
                
                delay_gauche = self._calculate_delay(max(self.MIN_SPEED_KMH, current_speed_gauche))
                delay_droit = self._calculate_delay(max(self.MIN_SPEED_KMH, current_speed_droit))
                
                self.motor_gauche.set_speed(delay_gauche)
                self.motor_droit.set_speed(delay_droit)
            
            time.sleep(0.1)
        
        # Arrêt
        self.stop_event.set()
        thread_gauche.join()
        thread_droit.join()
        
        print(f"Virage large terminé")

    # ... (le reste de tes fonctions restent identiques)