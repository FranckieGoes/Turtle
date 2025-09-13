# -*- coding: utf-8 -*-
import pigpio
import time
import threading
import sys

class StepperMotor:
    """
    Classe pour contrôler un moteur pas à pas à l'aide d'un driver TB6600
    connecté à un Raspberry Pi, en utilisant la bibliothèque pigpio.
    """
    def __init__(self, pi, dir_pin, step_pin, en_pin, delay=0.001, steps_per_rev=400, adjusted_steps_per_rev=1600):
        """
        Initialise le moteur et les broches GPIO.

        Args:
            pi (pigpio.pi): L'instance de la classe pigpio.
            dir_pin (int): Le numéro de la broche GPIO pour la direction.
            step_pin (int): Le numéro de la broche GPIO pour les pas.
            en_pin (int): Le numéro de la broche GPIO pour l'activation.
            delay (float): Le délai en secondes entre chaque impulsion (définit la vitesse).
            steps_per_rev (int): Le nombre de pas par tour du moteur.
            adjusted_steps_per_rev (int): Le nombre de pas par tour avec la micro-pas.
        """
        self.pi = pi
        self.DIR_PIN = dir_pin
        self.STEP_PIN = step_pin
        self.EN_PIN = en_pin
        self.DELAY = delay
        self.STEPS_PER_REV = steps_per_rev
        self.STEPS_PER_ADJUSTED_REV = adjusted_steps_per_rev
        self.current_thread = None

        # Configurer les broches en sortie
        self.pi.set_mode(self.DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.STEP_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.EN_PIN, pigpio.OUTPUT)

        # Désactiver le moteur par défaut
        self.disable()
        print(f"Moteur initialisé sur les broches DIR:{self.DIR_PIN}, STEP:{self.STEP_PIN}, EN:{self.EN_PIN}")

    def set_speed(self, delay):
        """
        Définit le délai entre les pas, ce qui change la vitesse.
        Note: C'est le délai pour le PWM, donc 2 * DELAY = période.
        """
        self.DELAY = delay

    def enable(self):
        """
        Active le driver du moteur pas à pas.
        La broche EN est active à l'état bas.
        """
        self.pi.write(self.EN_PIN, 0)
        time.sleep(0.01) # Petit délai pour la stabilité

    def disable(self):
        """
        Désactive le driver du moteur pas à pas.
        """
        self.pi.write(self.EN_PIN, 1)
        time.sleep(0.01) # Petit délai pour la stabilité

    def step(self, direction):
        """
        Génère une impulsion sur la broche STEP.
        
        Args:
            direction (str): 'forward' ou 'backward'.
        """
        if direction == 'forward':
            self.pi.write(self.DIR_PIN, 0)
        else:
            self.pi.write(self.DIR_PIN, 1)

        self.pi.write(self.STEP_PIN, 1)
        time.sleep(self.DELAY)
        self.pi.write(self.STEP_PIN, 0)
        time.sleep(self.DELAY)

    def move_continuously(self, direction, stop_event, pause_event):
        """
        Déplace le moteur en continu en utilisant le PWM matériel de pigpio.
        
        Args:
            direction (str): 'forward' ou 'backward'.
            stop_event (threading.Event): Événement pour arrêter le mouvement.
            pause_event (threading.Event): Événement pour mettre en pause le mouvement.
        """
        # S'assurer que le moteur est activé
        self.enable()

        # Configurer la direction
        if direction == 'forward':
            self.pi.write(self.DIR_PIN, 0)
        else:
            self.pi.write(self.DIR_PIN, 1)

        print(f"Mouvement continu démarré sur la broche {self.STEP_PIN}...")
        
        while not stop_event.is_set():
            # Si le robot est en pause, attendre
            if pause_event.is_set():
                # On arrête le PWM pour économiser l'énergie et éviter les vibrations
                self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
                pause_event.wait()
                if not stop_event.is_set():
                    # Reprendre le PWM à la bonne fréquence
                    # Frequence = 1 / (2 * DELAY)
                    freq = 1.0 / (2 * self.DELAY)
                    self.pi.hardware_PWM(self.STEP_PIN, int(freq), 500000)
            
            # S'assurer que le délai est positif pour éviter une fréquence infinie
            if self.DELAY > 0:
                freq = 1.0 / (2 * self.DELAY)
                # On met le cycle à 50% (500000)
                self.pi.hardware_PWM(self.STEP_PIN, int(freq), 500000)
            
            time.sleep(0.1)
            
        self.disable()
        self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
        print(f"Arrêt du mouvement continu sur la broche {self.STEP_PIN}.")

    def start_moving(self, direction, stop_event, pause_event):
        """
        Crée un thread pour déplacer le moteur en continu.
        """
        # S'assurer qu'un seul thread est actif pour ce moteur
        if self.current_thread and self.current_thread.is_alive():
            print("Un thread de mouvement est déjà en cours.")
            return

        self.enable()
        self.current_thread = threading.Thread(target=self.move_continuously, args=(direction, stop_event, pause_event))
        self.current_thread.start()
    
    def stop_moving(self):
        """
        Arrête le thread de mouvement en cours.
        """
        self.disable()
        self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
        print("Mouvement du moteur arrêté via pigpio.")

    def cleanup(self):
        """Arrête le moteur et nettoie le GPIO."""
        self.disable()
        self.pi.hardware_PWM(self.STEP_PIN, 0, 0) # Assurer l'arrêt du PWM
        print("Nettoyage du moteur pas à pas...")
