# -*- coding: utf-8 -*-
import math

class SpeedConverter:
    """
    Classe pour convertir une vitesse en km/h en nombre de pas moteur par seconde.
    """
    
    def __init__(self, steps_per_rev, wheel_diameter_mm):
        """
        Initialise le convertisseur avec les spécifications du moteur et de la roue.

        Args:
            steps_per_rev (int): Nombre de pas par tour du moteur.
            wheel_diameter_mm (float): Diamètre de la roue en millimètres.
        """
        self.steps_per_rev = steps_per_rev
        self.wheel_diameter_mm = wheel_diameter_mm
        
    def convert_kmh_to_steps_per_sec(self, speed_kmh):
        """
        Convertit une vitesse en km/h en pas par seconde.

        Args:
            speed_kmh (float): Vitesse en kilomètres par heure (km/h).
            
        Returns:
            float: Le nombre de pas par seconde requis.
        """
        # S'assurer que la vitesse est positive
        if speed_kmh < 0:
            speed_kmh = 0
            
        # 1. Calculer la circonférence de la roue en cm
        # Divise par 10 pour convertir mm en cm
        wheel_diameter_cm = self.wheel_diameter_mm / 10
        wheel_circumference_cm = math.pi * wheel_diameter_cm
        
        # 2. Convertir la vitesse de km/h en cm/s
        # 1 km = 100 000 cm, 1 h = 3600 s
        speed_cm_per_sec = (speed_kmh * 100000) / 3600
        
        # 3. Calculer les tours de roue par seconde
        wheel_revs_per_sec = speed_cm_per_sec / wheel_circumference_cm
        
        # 4. Convertir les tours de roue en pas par seconde
        steps_per_sec = wheel_revs_per_sec * self.steps_per_rev
        
        return steps_per_sec

# Exemple d'utilisation de la classe
if __name__ == "__main__":
    # Spécifications du robot :
    # Moteur avec 400 pas par tour (full step)
    # Roue de 61 mm de diamètre
    steps_per_revolution = 400
    wheel_diameter = 61.0
    
    converter = SpeedConverter(steps_per_revolution, wheel_diameter)
    
    # Tester une vitesse de 0.5 km/h
    test_speed_kmh = 0.5
    steps_per_second = converter.convert_kmh_to_steps_per_sec(test_speed_kmh)
    print(f"Pour une vitesse de {test_speed_kmh} km/h, le moteur doit tourner à {steps_per_second:.2f} pas/seconde.")

    # Tester une vitesse de 5 km/h
    test_speed_kmh = 5.0
    steps_per_second = converter.convert_kmh_to_steps_per_sec(test_speed_kmh)
    print(f"Pour une vitesse de {test_speed_kmh} km/h, le moteur doit tourner à {steps_per_second:.2f} pas/seconde.")

    # Tester une vitesse de 10 km/h
    test_speed_kmh = 10.0
    steps_per_second = converter.convert_kmh_to_steps_per_sec(test_speed_kmh)
    print(f"Pour une vitesse de {test_speed_kmh} km/h, le moteur doit tourner à {steps_per_second:.2f} pas/seconde.")
