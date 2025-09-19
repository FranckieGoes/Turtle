# Fichier : stepper_motor_class.py (MIS À JOUR)

import pigpio
import time
import threading

class StepperMotor:
    def __init__(self, pi, dir_pin, step_pin, en_pin):
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
        self.DELAY = 0.001 # Délai initial, sera mis à jour
        self.pi.set_mode(self.DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.STEP_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.EN_PIN, pigpio.OUTPUT)
        self.disable()
        print(f"Moteur sur la broche {self.STEP_PIN} désactivé.")

    def set_speed(self, delay):
        self.DELAY = delay

    def enable(self):
        self.pi.write(self.EN_PIN, 0)

    def disable(self):
        self.pi.write(self.EN_PIN, 1)

    def move_continuously(self, direction, stop_event, pause_event):
        print(f"Démarrage du mouvement continu sur la broche {self.STEP_PIN}...")
        self.enable()
        self.pi.write(self.DIR_PIN, 0 if direction == 'forward' else 1)
        
        while not stop_event.is_set():
            if pause_event.is_set():
                self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
                pause_event.wait()

            if self.DELAY > 0 and not pause_event.is_set():
                freq = 1.0 / (2 * self.DELAY)
                self.pi.hardware_PWM(self.STEP_PIN, int(freq), 500000)
            else:
                self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
            
            time.sleep(0.1)
            
        self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
        self.disable()
        print(f"Arrêt du mouvement continu sur la broche {self.STEP_PIN}.")

    # --- NOUVELLE FONCTION POUR LES MOUVEMENTS PRÉCIS ---
    def move_steps(self, steps, direction, delay):
        """
        Effectue un nombre précis de pas dans une direction donnée.
        C'est une fonction bloquante.
        """
        self.enable()
        self.pi.write(self.DIR_PIN, 0 if direction == 'forward' else 1)
        
        for _ in range(steps):
            self.pi.write(self.STEP_PIN, 1)
            time.sleep(delay)
            self.pi.write(self.STEP_PIN, 0)
            time.sleep(delay)

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
        self.pi.hardware_PWM(self.STEP_PIN, 0, 0)