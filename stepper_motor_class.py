# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
import threading

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
        self.current_thread = None # Garde une référence au thread en cours
        self.current_direction = True  # Nouvelle variable pour stocker la direction actuelle

        # Configuration de la carte Raspberry Pi
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.EN_PIN, GPIO.OUT)

        # Désactiver le moteur par défaut
        GPIO.output(self.EN_PIN, GPIO.HIGH)

    def enable(self):
        """Active le moteur."""
        GPIO.output(self.EN_PIN, GPIO.LOW)

    def disable(self):
        """Désactive le moteur."""
        GPIO.output(self.EN_PIN, GPIO.HIGH)
        print("Moteur désactivé.")

    def set_direction(self, direction):
        """
        Définit la direction de rotation et l'enregistre.
        
        Args:
            direction (bool): True pour un sens, False pour l'autre.
        """
        GPIO.output(self.DIR_PIN, direction)
        self.current_direction = direction  # Enregistre la direction

    def move_steps(self, steps, direction):
        """
        Fait avancer le moteur d'un nombre de pas spécifié.

        Args:
            steps (int): Le nombre de pas à effectuer.
            direction (bool): Le sens de rotation (True ou False).
        """
        self.set_direction(direction)
        self.enable()
        
        for _ in range(int(steps)):
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(self.DELAY)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(self.DELAY)

        self.disable()

    def step(self, direction):
        """Fait un seul pas dans la direction spécifiée."""
        self.set_direction(direction)
        GPIO.output(self.STEP_PIN, GPIO.HIGH)
        time.sleep(self.DELAY)
        GPIO.output(self.STEP_PIN, GPIO.LOW)
        time.sleep(self.DELAY)

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
        print(f"Vitesse mise à jour. Nouveau délai: {self.DELAY:.6f} secondes")

    def move_continuously(self, direction, stop_event, pause_event):
        """
        Fait tourner le moteur en continu jusqu'à ce qu'un événement d'arrêt soit déclenché.
        """
        self.set_direction(direction)
        self.enable()
        
        while not stop_event.is_set():
            if not pause_event.is_set():
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                time.sleep(self.DELAY)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                time.sleep(self.DELAY)
            else:
                time.sleep(0.1)
        
        self.disable()
    
    def ramp_up(self, target_delay, direction, stop_event, pause_event, ramp_steps=100):
        """
        Démarre le moteur progressivement.
        """
        start_delay = 0.005 # Délai initial pour une vitesse très basse
        delay_step_size = (start_delay - target_delay) / ramp_steps
        
        self.set_direction(direction)
        self.enable()
        
        current_delay = start_delay
        
        # Phase de rampe de montée
        for i in range(ramp_steps):
            if stop_event.is_set():
                break
            
            self.DELAY = current_delay
            self.step(direction)
            current_delay -= delay_step_size
        
        # Mouvement continu à la vitesse de croisière
        if not stop_event.is_set():
            self.DELAY = target_delay
            self.move_continuously(direction, stop_event, pause_event)
            
    def ramp_down(self, direction, stop_event, ramp_steps=100):
        """
        Arrête le moteur progressivement en augmentant le délai entre les pas.
        """
        current_delay = self.DELAY
        final_delay = 0.005  # Délai final pour un mouvement très lent
        
        # S'assurer que le délai final est plus grand pour ralentir
        if current_delay < final_delay:
            delay_step_size = (final_delay - current_delay) / ramp_steps
            
            for _ in range(ramp_steps):
                if stop_event.is_set():
                    break
                
                self.DELAY += delay_step_size
                self.step(direction)
            
        # Arrêt final une fois le ralentissement terminé
        self.disable()
        stop_event.set()