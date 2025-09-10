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
        
        # État de l'initialisation GPIO
        self.gpio_initialized = False

        # Configuration de la carte Raspberry Pi
        GPIO.setmode(GPIO.BCM)
        
        # Configuration des broches
        GPIO.setup(self.DIR_PIN, GPIO.OUT)
        GPIO.setup(self.STEP_PIN, GPIO.OUT)
        GPIO.setup(self.EN_PIN, GPIO.OUT)
        
        # Initialisation réussie
        self.gpio_initialized = True
        
    def set_direction(self, direction):
        """Définit la direction du moteur (True pour une direction, False pour l'autre)."""
        if self.gpio_initialized:
            GPIO.output(self.DIR_PIN, direction)
        
    def enable(self):
        """Active le moteur. La broche EN doit être LOW pour activer."""
        if self.gpio_initialized:
            print(f"Moteur sur broche EN {self.EN_PIN} activé.")
            GPIO.output(self.EN_PIN, GPIO.LOW)
            
    def disable(self):
        """Désactive le moteur. La broche EN doit être HIGH pour désactiver."""
        if self.gpio_initialized:
            print(f"Moteur sur broche EN {self.EN_PIN} désactivé.")
            GPIO.output(self.EN_PIN, GPIO.HIGH)
        
    def step(self, direction):
        """Effectue un pas du moteur."""
        if self.gpio_initialized:
            self.set_direction(direction)
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(self.DELAY)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(self.DELAY)
            
    def move_steps(self, steps, direction):
        """
        Fait tourner le moteur d'un nombre de pas spécifié.
        
        Args:
            steps (int): Nombre de pas à effectuer.
            direction (bool): Direction de la rotation.
            delay (float): Délai entre les pas pour contrôler la vitesse.
        """
        if self.gpio_initialized:
            self.set_direction(direction)
            self.enable()
            print(f"Déplacement de {steps} pas. Direction: {direction}, Délai: {delay:.6f}s")
            for _ in range(steps):
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                time.sleep(self.DELAY) # Use the delay passed as a parameter
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                time.sleep(self.DELAY)
            self.disable()
            print("Déplacement terminé.")
            
    def move_degrees(self, degrees, direction):
        """
        Fait tourner le moteur d'un angle spécifié.
        
        Args:
            degrees (float): Angle de rotation en degrés.
            direction (bool): Direction de la rotation.
        """
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

        Args:
            direction (bool): La direction de la rotation (True ou False).
            stop_event (threading.Event): Événement pour arrêter le mouvement.
            pause_event (threading.Event): Événement pour mettre en pause le mouvement.
        """
        if not self.gpio_initialized:
            print("Impossible de démarrer le mouvement continu. Les GPIO n'ont pas été initialisés correctement.")
            return

        self.set_direction(direction)
        self.enable()
        
        while not stop_event.is_set():
            if not pause_event.is_set():
                GPIO.output(self.STEP_PIN, GPIO.HIGH)
                time.sleep(self.DELAY)
                GPIO.output(self.STEP_PIN, GPIO.LOW)
                time.sleep(self.DELAY)
            else:
                time.sleep(0.1)  # Attendre un peu si en pause pour ne pas surcharger le CPU
        
        self.disable()
        
    def start_thread(self, target, args):
        """Démarre un thread pour l'exécution du mouvement en continu."""
        if self.current_thread and self.current_thread.is_alive():
            print("Un thread est déjà en cours d'exécution pour ce moteur. Arrêt du thread précédent.")
            # Le thread précédent s'arrêtera de lui-même si un stop_event est déjà défini
        
        self.current_thread = threading.Thread(target=target, args=args, daemon=True)
        self.current_thread.start()

    def ramp_up(self, direction, stop_event, pause_event, target_delay, ramp_steps=100):
        """
        Démarre le moteur progressivement avec une rampe de montée.
        
        Args:
            direction (bool): Direction de la rotation.
            stop_event (threading.Event): Événement d'arrêt.
            pause_event (threading.Event): Événement de pause.
            target_delay (float): Le délai de la vitesse de croisière.
            ramp_steps (int): Nombre de pas pour la rampe de montée.
        """
        if not self.gpio_initialized:
            print("Impossible de démarrer le mouvement en rampe. Les GPIO n'ont pas été initialisés correctement.")
            return

        print("Démarrage du mouvement avec une rampe de montée.")
        
        start_delay = 0.01 # Délai de départ lent
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
        if not self.gpio_initialized:
            print("Les GPIO ne sont pas initialisés, le moteur est déjà désactivé.")
            return

        print("Démarrage de la rampe de descente...")
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

            self.disable()
            print("Rampe de descente terminée. Moteur désactivé.")
