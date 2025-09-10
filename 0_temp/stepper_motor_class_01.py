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
        self.current_thread = None # Garde une référence au thread de mouvement continu
        self.stop_event = threading.Event()
        self.pause_event = threading.Event()

        # Configuration de la carte Raspberry Pi
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.DIR_PIN, GPIO.OUT)
            GPIO.setup(self.STEP_PIN, GPIO.OUT)
            GPIO.setup(self.EN_PIN, GPIO.OUT)
            self.disable() # S'assurer que le moteur est désactivé au démarrage
            self.gpio_initialized = True
        except Exception as e:
            print(f"Avertissement: Impossible d'initialiser les GPIO. L'exécution sans matériel physique est possible. Erreur: {e}")
            self.gpio_initialized = False

    def enable(self):
        """Active le moteur."""
        if self.gpio_initialized:
            GPIO.output(self.EN_PIN, GPIO.LOW) # Le pin EN est généralement actif à l'état bas
        print("Moteur activé.")

    def disable(self):
        """Désactive le moteur."""
        if self.gpio_initialized:
            GPIO.output(self.EN_PIN, GPIO.HIGH) # Le pin EN est généralement actif à l'état bas
        print("Moteur désactivé.")

    def set_direction(self, clockwise):
        """
        Définit la direction de la rotation.

        Args:
            clockwise (bool): True pour le sens horaire, False pour le sens anti-horaire.
        """
        if self.gpio_initialized:
            GPIO.output(self.DIR_PIN, clockwise)
        print(f"Direction définie: {'Horaire' if clockwise else 'Anti-horaire'}")

    def step(self, direction):
        """
        Effectue un seul pas du moteur dans la direction spécifiée.
        """
        self.set_direction(direction)
        if self.gpio_initialized:
            GPIO.output(self.STEP_PIN, GPIO.HIGH)
            time.sleep(self.DELAY)
            GPIO.output(self.STEP_PIN, GPIO.LOW)
            time.sleep(self.DELAY)

    def move_steps(self, steps, delay_sec):
        """
        Fait tourner le moteur d'un nombre de pas spécifique à une vitesse donnée.
        Cette méthode est bloquante.
        """
        if self.current_thread and self.current_thread.is_alive():
            print("Un mouvement continu est déjà en cours. Impossible d'exécuter move_steps.")
            return

        self.DELAY = delay_sec
        self.enable()
        print(f"Déplacement de {steps} pas avec un délai de {self.DELAY} s...")

        for _ in range(steps):
            self.step(GPIO.HIGH) # Direction peu importe car les deux moteurs sont contrôlés par la classe parente
        
        self.disable()
        print("Déplacement terminé.")


    def move_continuously(self, direction, stop_event, pause_event):
        """
        Fait tourner le moteur en continu jusqu'à ce qu'un événement d'arrêt soit déclenché.
        """
        self.set_direction(direction)
        self.enable()
        print(f"Démarrage du mouvement continu. Direction: {'Horaire' if direction else 'Anti-horaire'}")
        
        while not stop_event.is_set():
            if not pause_event.is_set():
                if self.gpio_initialized:
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    time.sleep(self.DELAY)
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    time.sleep(self.DELAY)
                else:
                    # Simulation pour les tests
                    time.sleep(self.DELAY * 2)
            else:
                time.sleep(0.1)  # Délai pendant la pause pour ne pas surcharger le processeur
        
        self.disable()
        print("Mouvement continu arrêté.")

    def move_for_duration(self, duration_sec, direction, stop_event, pause_event):
        """
        Fait tourner le moteur pendant une durée spécifique en utilisant un thread.
        """
        self.stop_event = stop_event
        self.pause_event = pause_event
        
        self.set_direction(direction)
        self.enable()
        
        start_time = time.time()
        while (time.time() - start_time) < duration_sec and not self.stop_event.is_set():
            if not self.pause_event.is_set():
                if self.gpio_initialized:
                    GPIO.output(self.STEP_PIN, GPIO.HIGH)
                    time.sleep(self.DELAY)
                    GPIO.output(self.STEP_PIN, GPIO.LOW)
                    time.sleep(self.DELAY)
                else:
                    time.sleep(self.DELAY * 2)
            else:
                time.sleep(0.1)
                
        self.stop_event.set()
        self.disable()


    def move_degrees(self, degrees, direction):
        """
        Fait tourner le moteur d'un nombre de degrés précis.
        """
        steps_to_move = int(self.STEPS_PER_ADJUSTED_REV * (degrees / 360))
        print(f"Rotation de {degrees} degrés, soit {steps_to_move} pas.")
        self.move_steps(steps_to_move, direction)

    def set_speed(self, delay_sec):
        """
        Ajuste la vitesse du moteur en modifiant le délai entre les impulsions.
        """
        self.DELAY = delay_sec
        print(f"Vitesse mise à jour. Nouveau délai: {self.DELAY:.6f} secondes")

    def __del__(self):
        """
        S'assure que les broches GPIO sont nettoyées à la destruction de l'objet.
        """
        if self.gpio_initialized:
            GPIO.cleanup()
            print("GPIOs nettoyés.")

    def ramp_up(self, direction, target_delay, ramp_steps=100, start_delay=0.01):
        """
        Accélère le moteur progressivement en diminuant le délai entre les pas.
        """
        delay_step_size = (start_delay - target_delay) / ramp_steps
        
        self.set_direction(direction)
        self.enable()
        
        current_delay = start_delay
        
        # Phase de rampe de montée
        for _ in range(ramp_steps):
            self.DELAY = current_delay
            self.step(direction)
            current_delay -= delay_step_size
        
        # Mouvement continu à la vitesse de croisière
        self.DELAY = target_delay
        self.move_continuously(direction, self.stop_event, self.pause_event)

    def ramp_down(self, direction, ramp_steps=100, final_delay=0.005):
        """
        Arrête le moteur progressivement en augmentant le délai entre les pas.
        """
        current_delay = self.DELAY
        
        # S'assurer que le délai final est plus grand pour ralentir
        if current_delay < final_delay:
            delay_step_size = (final_delay - current_delay) / ramp_steps
            
            for _ in range(ramp_steps):
                self.DELAY += delay_step_size
                self.step(direction)
        
        self.disable()

if __name__ == "__main__":
    try:
        # Remplacez ces valeurs par les broches GPIO de votre configuration
        MOTOR_DIR_PIN = 27
        MOTOR_STEP_PIN = 17
        MOTOR_EN_PIN = 22

        # Création d'une instance de la classe
        motor = StepperMotor(MOTOR_DIR_PIN, MOTOR_STEP_PIN, MOTOR_EN_PIN)

        # Exemple d'utilisation
        print("Test de mouvement: 1000 pas en sens horaire.")
        motor.move_steps(1000, 0.001)
        time.sleep(2)

        print("\nTest de mouvement: 1000 pas en sens anti-horaire.")
        motor.set_direction(False)
        motor.move_steps(1000, 0.001)
        time.sleep(2)
        
        # Test de mouvement continu dans un thread
        print("\nTest de mouvement continu pour 5 secondes.")
        stop_flag = threading.Event()
        pause_flag = threading.Event()
        motor_thread = threading.Thread(target=motor.move_for_duration, args=(5, True, stop_flag, pause_flag))
        motor_thread.start()
        motor_thread.join() # Attendre que le thread se termine
        
    except KeyboardInterrupt:
        print("\nArrêt du script par l'utilisateur.")
    finally:
        # Le destructeur __del__ est appelé automatiquement, mais un appel explicite est une bonne pratique
        # en cas de besoin de nettoyage immédiat.
        pass
