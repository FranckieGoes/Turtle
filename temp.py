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

        # Configuration des broches
        self.pi.set_mode(self.DIR_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.STEP_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.EN_PIN, pigpio.OUTPUT)

    def set_dir(self, direction):
        """Définit la direction du moteur (forward/backward)."""
        if direction == 'forward':
            self.pi.write(self.DIR_PIN, pigpio.HIGH)
        elif direction == 'backward':
            self.pi.write(self.DIR_PIN, pigpio.LOW)

    def enable(self):
        """Active le moteur."""
        self.pi.write(self.EN_PIN, pigpio.LOW)

    def disable(self):
        """Désactive le moteur."""
        self.pi.write(self.EN_PIN, pigpio.HIGH)

    def set_speed(self, delay):
        """Définit le délai entre les pas pour ajuster la vitesse."""
        self.DELAY = delay

    def get_speed(self):
        """Retourne le délai actuel."""
        return self.DELAY

    def move_steps(self, steps):
        """
        Fait bouger le moteur pour un nombre de pas précis.
        Utilise pigpio pour un contrôle précis via hardware PWM.
        """
        if steps <= 0:
            return
            
        self.enable()
        
        # Le PWM a une fréquence en Hz (pas/seconde). Le délai est en secondes/pas.
        # Donc, freq = 1 / (2 * delay) car un cycle PWM correspond à deux états (HIGH et LOW)
        try:
            freq = 1.0 / (2 * self.DELAY)
            self.pi.hardware_PWM(self.STEP_PIN, int(freq), 500000)
            
            # Attendre que le nombre de pas soit atteint
            time_to_move = steps * self.DELAY * 2 # 2 pour le cycle PWM
            time.sleep(time_to_move)

        except ZeroDivisionError:
            print("Erreur : La vitesse est infinie, impossible de démarrer le moteur.")
        finally:
            # Arrêter le PWM et désactiver le moteur après la rotation
            self.pi.hardware_PWM(self.STEP_PIN, 0, 0)
            self.disable()

    def cleanup(self):
        """Arrête le moteur et nettoie les broches GPIO."""
        self.disable()
        self.pi.stop()
