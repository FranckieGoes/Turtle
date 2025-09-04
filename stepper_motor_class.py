# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time

class StepperMotor:
    """
    Classe pour contrôler un moteur pas à pas à l'aide d'un driver TB6600
    connecté à un Raspberry Pi.
    """
    def __init__(self, dir_pin, step_pin, en_pin, delay=0.001, steps_per_rev=400, adjusted_steps_per_rev=1600):
        """
        Initialise le moteur et les broches GPIO.

        Args:
            dir_pin (int): Le numéro de la broche GPIO pour la direction.
            step_pin (int): Le numéro de la broche GPIO pour les pas.
            en_pin (int): Le numéro de la broche GPIO pour l'activation.
            delay (float): Le délai en secondes entre chaque impulsion (définit la vitesse).
            steps_per_rev (int): Le nombre de pas par tour du moteur.
            adjusted_steps_per_rev (int): Le nombre de pas par tour avec la micro-pas.
        """
        self.DIR_PIN = dir_pin
        self.STEP_PIN = step_pin
        self.EN_PIN = en_pin
        self.DELAY = delay
        self.STEPS_PER_REV = steps_per_rev
        self.STEPS_PER_ADJUSTED_REV = adjusted_steps_per_rev

        # Configuration de la carte Raspberry Pi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.EN_PIN, GPIO.OUT)
        
        # Le driver TB6600 est activé par un signal LOW
        GPIO.output(self.EN_PIN, GPIO.LOW)
        print("Driver du moteur activé.")

    def set_direction(self, direction):
        """Définit la direction du moteur (True pour une direction, False pour l'autre)."""
        GPIO.output(self.DIR_PIN, direction)

    def move_steps(self, steps, direction):
        """
        Fait avancer le moteur d'un certain nombre de pas.

        Args:
            steps (int): Le nombre de pas à effectuer.
            direction (bool): Le sens de rotation (True ou False).
        """
        self.set_direction(direction)
        time.sleep(0.05) # Petite pause pour s'assurer que le driver a changé de direction

        # Démarrer avec un délai élevé (vitesse lente)
        current_delay = self.DELAY * 10
        acceleration = 0.00001
        min_delay = self.DELAY /10
        print(f"min_delay {min_delay:.6f}")
        half_steps = steps / 2
        
        print(f"Déplacement de {steps} pas avec accélération et décélération...")
        #for _ in range(steps):
        for i in range(steps):
             # Phase d'accélération
            if i < half_steps and current_delay > min_delay:
                current_delay -= acceleration
                #print("current_delay", current_delay)
            # Phase de décélération
            elif i >= half_steps and current_delay < self.DELAY * 5:
                current_delay += acceleration / 20
                #print(f"current_delay {current_delay:.9f}")
            # S'assurer que le délai ne descend pas en dessous du minimum
            if current_delay < min_delay:
                current_delay = min_delay

            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            #time.sleep(self.DELAY)
            time.sleep(current_delay)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            #time.sleep(self.DELAY)
            time.sleep(current_delay)

    def rotate_degrees(self, degrees, direction):
        """
        Fait tourner le moteur d'un certain nombre de degrés.

        Args:
            degrees (int): Le nombre de degrés à tourner.
            direction (bool): Le sens de rotation (True ou False).
        """
        # Calcule le nombre de pas requis pour un certain nombre de degrés
        steps_to_move = int(self.STEPS_PER_ADJUSTED_REV * (degrees / 360))
        print(f"Rotation de {degrees} degrés, soit {steps_to_move} pas.")
        self.move_steps(steps_to_move, direction)

    def set_speed(self, delay_sec):
        """
        Ajuste la vitesse du moteur en modifiant le délai entre les impulsions.
        
        Args:
            delay_sec (float): Le nouveau délai en secondes. Plus la valeur est petite, plus le moteur tourne vite.
        """
        self.DELAY = delay_sec
        print(f"Vitesse mise à jour. Nouveau délai: {self.DELAY:.9f}s")    
        
    def disable_driver(self):
        """Désactive le driver pour couper l'alimentation du moteur."""
        GPIO.output(self.EN_PIN, GPIO.HIGH)
        print("Driver désactivé.")
