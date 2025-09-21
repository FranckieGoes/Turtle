# Modifications à apporter à robot_controller_class.py

class RobotController:
    # ... (toutes tes constantes et __init__ restent identiques)
    
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