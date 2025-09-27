from shapely.geometry import Polygon, LinearRing # version évoluée

def creer_perimetres_concentriques(points_polygone_initial, delta, nombre_perimetres):
    """
    Crée une série de périmètres concentriques vers l'intérieur d'un polygone.

    :param points_polygone_initial: Une liste de tuples (x, y) définissant le polygone extérieur.
    :param delta: La distance de décalage (positive) entre chaque périmètre (en mètres).
    :param nombre_perimetres: Le nombre de périmètres intérieurs à générer.
    :return: Une liste de listes de coordonnées. Chaque sous-liste représente un périmètre.
    """
    # Crée un objet Polygon à partir des points
    try:
        polygone_initial = Polygon(points_polygone_initial)
    except Exception as e:
        print(f"Erreur lors de la création du polygone : {e}")
        return []

    # Vérifie si le polygone est valide
    if not polygone_initial.is_valid:
        print("Le polygone initial n'est pas valide (il se croise probablement).")
        return []

    liste_perimetres = []
    
    print(f"Génération de {nombre_perimetres} périmètre(s) avec un delta de {delta} m.")

    for i in range(1, nombre_perimetres + 1):
        # Pour un décalage vers l'intérieur, on utilise une distance négative
        distance_decalage = -delta * i
        
        # La méthode buffer() fait tout le travail complexe
        perimetre_interieur = polygone_initial.buffer(distance_decalage)
        
        # Si le polygone devient trop petit, il peut disparaître.
        if perimetre_interieur.is_empty:
            print(f"Arrêt à l'itération {i} : le polygone est devenu trop petit pour continuer.")
            break
        
        # On récupère les coordonnées du nouveau périmètre
        # .exterior.coords[:-1] pour obtenir la liste des points sans répéter le premier à la fin
        coords = list(perimetre_interieur.exterior.coords)
        liste_perimetres.append(coords)

    return liste_perimetres

# --- Exemple d'utilisation ---
if __name__ == "__main__":
    # Définissez ici les coordonnées GPS ou locales de votre terrain.
    # C'est un exemple de polygone en forme de "L".
    # 
    mon_terrain = [
        (0, 0), 
        (20, 0), 
        (20, 10), 
        (10, 10), 
        (10, 20), 
        (0, 20),
        (0, 0) # Optionnel, Shapely le ferme automatiquement
    ]

    # Paramètres pour la tonte
    LARGEUR_DE_TONTE = 0.5  # Le "delta" sera la largeur de coupe du robot
    NOMBRE_DE_PASSAGES = 5  # Pour créer une bande de 5 * 0.5m = 2.5m

    # Générer les trajectoires
    trajectoires_concentriques = creer_perimetres_concentriques(
        points_polygone_initial=mon_terrain,
        delta=LARGEUR_DE_TONTE,
        nombre_perimetres=NOMBRE_DE_PASSAGES
    )

    # Afficher les résultats
    if trajectoires_concentriques:
        for i, perimetre in enumerate(trajectoires_concentriques):
            print(f"\n--- Périmètre {i+1} ---")
            # On arrondit pour un affichage plus propre
            perimetre_arrondi = [(round(x, 2), round(y, 2)) for x, y in perimetre]
            print(perimetre_arrondi)







import time # version simple

# --- Fonctions de contrôle moteur (À REMPLACER) ---
# Remplacez le contenu de ces fonctions par le code qui contrôle VOS moteurs.
# Par exemple, en utilisant les bibliothèques RPi.GPIO ou gpiozero.

def avancer(distance_m):
    """Fait avancer le robot d'une certaine distance en mètres."""
    # Exemple : calculer le temps nécessaire en fonction de la vitesse du robot
    vitesse_ms = 0.5  # Vitesse du robot en mètres/seconde (à calibrer)
    temps_necessaire = distance_m / vitesse_ms
    print(f"  -> AVANCE de {distance_m:.2f} m (durée: {temps_necessaire:.2f} s)")
    # time.sleep(temps_necessaire) # Décommenter pour une simulation réelle

def tourner_gauche_90():
    """Fait tourner le robot de 90 degrés sur sa gauche."""
    temps_rotation = 2 # Temps en secondes pour un virage à 90° (à calibrer)
    print("  -> TOURNE à gauche de 90°")
    # time.sleep(temps_rotation) # Décommenter pour une simulation réelle

# --- Fonction principale pour la tonte concentrique ---

def tonte_concentrique_spiral(longueur_zone, largeur_zone, largeur_tonte, nombre_tours):
    """
    Réalise une tonte en spirale vers l'intérieur.
    
    :param longueur_zone: Longueur totale de la zone de tonte (en mètres).
    :param largeur_zone: Largeur totale de la zone de tonte (en mètres).
    :param largeur_tonte: Largeur de coupe du robot (en mètres).
    :param nombre_tours: Le nombre de tours concentriques à effectuer.
    """
    print(f"--- Début de la tonte concentrique pour {nombre_tours} tour(s) ---")
    
    longueur_segment = longueur_zone
    largeur_segment = largeur_zone
    
    # Un "tour" complet est composé de 4 segments (2 longueurs, 2 largeurs)
    # Mais la logique en spirale réduit la longueur de 2 segments par tour.
    # Nous effectuons donc 2*nombre_tours paires de mouvements.
    for i in range(nombre_tours * 2):
        
        # Vérification de sécurité pour ne pas tondre en négatif si la zone est petite
        if longueur_segment <= 0 or largeur_segment <= 0:
            print("INFO: La zone restante est trop petite pour continuer la spirale.")
            break
            
        print(f"\n--- Cycle {i+1}/{nombre_tours*2} ---")
        
        # Premier segment (longueur)
        avancer(longueur_segment)
        tourner_gauche_90()
        
        # Le segment suivant (largeur) sera plus court d'une largeur de tonte
        largeur_segment -= largeur_tonte
        if largeur_segment <= 0: break # Sortie anticipée
        
        # Deuxième segment (largeur)
        avancer(largeur_segment)
        tourner_gauche_90()
        
        # Le segment suivant (longueur) sera également plus court
        longueur_segment -= largeur_tonte

    print("\n--- Tonte concentrique terminée ---")
    largeur_bande_tondue = nombre_tours * largeur_tonte
    print(f"Une bande d'environ {largeur_bande_tondue:.2f} m a été dégagée sur le périmètre.")


# --- Exemple d'utilisation ---
if __name__ == "__main__":
    # Définissez ici les paramètres de votre zone et de votre robot
    ZONE_LONGUEUR = 20  # en mètres
    ZONE_LARGEUR = 10   # en mètres
    ROBOT_LARGEUR_TONTE = 0.5  # 50 cm
    
    # Pour créer une bande de 2 à 3 mètres :
    # Si largeur de tonte = 0.5m, il faut 4 à 6 tours.
    NB_TOURS_CONCENTRIQUES = 5 
    
    tonte_concentrique_spiral(
        longueur_zone=ZONE_LONGUEUR,
        largeur_zone=ZONE_LARGEUR,
        largeur_tonte=ROBOT_LARGEUR_TONTE,
        nombre_tours=NB_TOURS_CONCENTRIQUES
    )