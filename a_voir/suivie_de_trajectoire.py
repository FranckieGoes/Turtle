import math

# --- Fonctions de contrôle moteur de base (À REMPLACER) ---
# Celles-ci doivent être remplacées par le code qui pilote VOS moteurs.

def avancer(distance_m):
    """Fait avancer le robot d'une certaine distance."""
    # Votre code pour faire tourner les roues pendant une durée calculée
    print(f"  -> COMMANDE: AVANCER de {distance_m:.2f} m")

def tourner(angle_deg):
    """Fait tourner le robot sur place d'un certain angle en degrés."""
    # Votre code pour faire pivoter le robot.
    # Un angle positif signifie un virage à gauche (anti-horaire).
    # Un angle négatif signifie un virage à droite (horaire).
    direction = "gauche" if angle_deg > 0 else "droite"
    print(f"  -> COMMANDE: TOURNER de {abs(angle_deg):.2f}° vers la {direction}")

# --- Fonction de navigation principale ---

def suivre_trajectoire(trajectoire):
    """
    Pilote le robot pour qu'il suive une trajectoire définie par une liste de points.
    
    :param trajectoire: Une liste de tuples de coordonnées (x, y).
    """
    # État initial du robot
    pos_x, pos_y = trajectoire[0]
    # On suppose que le robot regarde initialement vers l'axe des X positifs (angle de 0 radian)
    heading_rad = 0.0  
    
    print(f"--- Début du suivi de trajectoire ---")
    print(f"Position de départ: ({pos_x:.2f}, {pos_y:.2f})")

    # On parcourt la trajectoire à partir du deuxième point
    for i in range(1, len(trajectoire)):
        cible_x, cible_y = trajectoire[i]
        print(f"\nProchaine cible: ({cible_x:.2f}, {cible_y:.2f})")

        # 1. Calcul du vecteur vers la cible
        vecteur_x = cible_x - pos_x
        vecteur_y = cible_y - pos_y

        # 2. Calcul de l'angle absolu vers la cible
        # math.atan2 gère tous les quadrants pour nous, c'est très pratique.
        angle_cible_rad = math.atan2(vecteur_y, vecteur_x)

        # 3. Calcul de l'angle de virage (différence entre l'angle cible et l'orientation actuelle)
        virage_rad = angle_cible_rad - heading_rad
        
        # Normaliser l'angle pour choisir le virage le plus court (-180° à 180°)
        if virage_rad > math.pi:
            virage_rad -= 2 * math.pi
        if virage_rad < -math.pi:
            virage_rad += 2 * math.pi
            
        # Conversion en degrés pour la commande moteur
        virage_deg = math.degrees(virage_rad)

        # 4. Calcul de la distance à parcourir
        distance = math.sqrt(vecteur_x**2 + vecteur_y**2)

        # 5. Envoyer les commandes au robot
        if abs(virage_deg) > 0.5: # On ne tourne pas pour des angles minuscules
            tourner(virage_deg)
        
        if distance > 0.01: # On n'avance pas pour des distances infimes
            avancer(distance)
        
        # 6. Mettre à jour l'état du robot pour la prochaine itération
        pos_x, pos_y = cible_x, cible_y
        heading_rad = angle_cible_rad
        
    print("\n--- Trajectoire terminée ---")


# --- Exemple d'utilisation ---
if __name__ == "__main__":
    # On reprend la première trajectoire générée dans le script précédent.
    # C'est le premier périmètre intérieur de notre terrain en "L".
    # (Coordonnées arrondies pour la lisibilité)
    perimetre_1 = [
        (0.5, 0.5), (19.5, 0.5), (19.5, 9.5), (10.5, 9.5), 
        (10.5, 19.5), (0.5, 19.5), (0.5, 0.5)
    ]

    suivre_trajectoire(perimetre_1)