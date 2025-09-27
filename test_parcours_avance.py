# -*- coding: utf-8 -*-
"""
OUTIL DE DIAGNOSTIC - Génération Parcours Avancé
À exécuter sur votre serveur pour identifier le problème exact
"""

import math
import json

# Simuler l'import Shapely (remplacez par votre vraie config)
try:
    from shapely.geometry import Polygon, LineString
    SHAPELY_AVAILABLE = True
    print("✓ Shapely disponible")
except ImportError:
    SHAPELY_AVAILABLE = False
    print("✗ Shapely non disponible")

def test_zone_04():
    """
    Test avec les données de votre zone_04
    Remplacez ces vecteurs par les vrais de votre zone_04
    """
    # EXEMPLE de vecteurs zone_04 (remplacez par les vrais)
    """zone_04_vectors = [
        {"distance": 0, "relative_angle": 0},      # début
        {"distance": 500, "relative_angle": 0},    # droite
        {"distance": 0, "relative_angle": 90},     # rotation
        {"distance": 300, "relative_angle": 0},    # haut
        {"distance": 0, "relative_angle": 90},     # rotation
        {"distance": 500, "relative_angle": 0},    # gauche
        {"distance": 0, "relative_angle": 90},     # rotation
        {"distance": 300, "relative_angle": 0},    # bas
        {"distance": 0, "relative_angle": 90},     # rotation retour
    ]"""
    """SELECT perimeter_vectors FROM zones WHERE name='zone_04';"""

    zone_04_vectors = [
        {"distance": 432.2554270426432, "relative_angle": 0},
        {"distance": 0, "relative_angle": 90.0},
        {"distance": 505.96491495768225, "relative_angle": 0},
        {"distance": 0, "relative_angle": 90.0},
        {"distance": 491.5692607561747, "relative_angle": 0},
        {"distance": 0, "relative_angle": 90.0},
        {"distance": 608.0036362012227, "relative_angle": 0},
        {"distance": 0, "relative_angle": 180.0}]
    
    print(f"=== TEST ZONE_04 ===")
    print(f"Vecteurs d'entrée: {len(zone_04_vectors)}")
    
    # Afficher tous les vecteurs
    for i, v in enumerate(zone_04_vectors):
        print(f"  {i}: distance={v.get('distance', 0)}cm, angle={v.get('relative_angle', 0)}°")
    
    # Test des différentes phases
    result = test_generate_advanced_mowing_path(zone_04_vectors, 20, 3)
    return result

def test_generate_advanced_mowing_path(perimeter_vectors, mower_width_cm, concentric_passes):
    """
    Version test de la génération avec logs détaillés
    """
    print(f"\n=== DÉBUT GÉNÉRATION PARCOURS AVANCÉ ===")
    print(f"Largeur coupe: {mower_width_cm}cm")
    print(f"Passes concentriques: {concentric_passes}")
    
    all_vectors = []
    
    # PHASE 1: PÉRIMÈTRE
    print(f"\n--- PHASE 1: PÉRIMÈTRE ---")
    all_vectors.extend(perimeter_vectors)
    print(f"✓ Périmètre ajouté: {len(perimeter_vectors)} vecteurs")
    
    # PHASE 2: TEST SHAPELY
    if not SHAPELY_AVAILABLE:
        print(f"\n--- PHASE 2: SHAPELY NON DISPONIBLE ---")
        print(f"✗ Arrêt - retour périmètre seulement")
        return all_vectors
    
    print(f"\n--- PHASE 2: CRÉATION POLYGONE ---")
    
    try:
        # Étape 2.1: Conversion vecteurs → points
        polygon_points = test_vectors_to_polygon_points(perimeter_vectors)
        print(f"✓ Points polygone: {len(polygon_points)}")
        
        if len(polygon_points) < 4:
            print(f"✗ Pas assez de points ({len(polygon_points)}) - arrêt")
            return all_vectors
        
        # Étape 2.2: Fermeture polygone
        if polygon_points[0] != polygon_points[-1]:
            polygon_points.append(polygon_points[0])
            print(f"✓ Polygone fermé - total: {len(polygon_points)} points")
        
        # Étape 2.3: Création polygone Shapely
        try:
            main_polygon = Polygon(polygon_points)
            print(f"✓ Polygone Shapely créé")
            print(f"  - Valide: {main_polygon.is_valid}")
            print(f"  - Aire: {main_polygon.area:.1f}cm²")
            print(f"  - Limites: {main_polygon.bounds}")
            
            if not main_polygon.is_valid:
                main_polygon = main_polygon.buffer(0)
                print(f"✓ Polygone corrigé - Valide: {main_polygon.is_valid}")
                
        except Exception as e:
            print(f"✗ Erreur création polygone: {e}")
            return all_vectors
        
        if main_polygon.is_empty or not main_polygon.is_valid:
            print(f"✗ Polygone invalide ou vide - arrêt")
            return all_vectors
            
        # PHASE 3: PASSES CONCENTRIQUES
        print(f"\n--- PHASE 3: PASSES CONCENTRIQUES ---")
        
        current_polygon = main_polygon
        concentric_vectors = []
        
        for pass_num in range(concentric_passes):
            print(f"\n  Passe {pass_num + 1}:")
            
            # Distance de rétrécissement
            shrink_distance = mower_width_cm * (pass_num + 1) * 0.85
            print(f"    Distance rétrécissement: -{shrink_distance:.1f}cm")
            
            try:
                # Buffer négatif
                inner_polygon = current_polygon.buffer(-shrink_distance)
                print(f"    Buffer effectué")
                print(f"    - Vide: {inner_polygon.is_empty}")
                if not inner_polygon.is_empty:
                    print(f"    - Aire: {inner_polygon.area:.1f}cm²")
                    print(f"    - Type: {type(inner_polygon)}")
                
                # Vérifier si vide
                if inner_polygon.is_empty:
                    print(f"    ✗ Polygone vide - arrêt passes")
                    break
                
                # Vérifier fragmentation
                if hasattr(inner_polygon, 'geoms'):
                    print(f"    - Multi-polygone: {len(inner_polygon.geoms)} fragments")
                    
                    # Trouver le plus grand
                    largest_area = 0
                    largest_poly = None
                    for i, geom in enumerate(inner_polygon.geoms):
                        if hasattr(geom, 'area'):
                            print(f"      Fragment {i}: {geom.area:.1f}cm²")
                            if geom.area > largest_area:
                                largest_area = geom.area
                                largest_poly = geom
                    
                    if largest_poly and largest_area > (mower_width_cm * 2) ** 2:
                        inner_polygon = largest_poly
                        print(f"    ✓ Fragment principal sélectionné: {largest_area:.1f}cm²")
                    else:
                        print(f"    ✗ Aucun fragment assez grand - arrêt passes")
                        break
                
                # Vérifier aire minimale
                min_area = (mower_width_cm * 2) ** 2
                if inner_polygon.area < min_area:
                    print(f"    ✗ Aire trop petite ({inner_polygon.area:.1f} < {min_area:.1f}cm²) - arrêt passes")
                    break
                
                # Extraire contour
                if hasattr(inner_polygon, 'exterior') and inner_polygon.exterior:
                    coords = list(inner_polygon.exterior.coords[:-1])
                    print(f"    ✓ Contour extrait: {len(coords)} points")
                    
                    if len(coords) >= 3:
                        # Alterner sens
                        if pass_num % 2 == 1:
                            coords.reverse()
                            print(f"    ✓ Sens inversé")
                        
                        # Convertir en vecteurs
                        pass_vectors = test_contour_to_vectors(coords)
                        
                        if pass_vectors:
                            concentric_vectors.extend(pass_vectors)
                            current_polygon = inner_polygon
                            print(f"    ✓ {len(pass_vectors)} vecteurs ajoutés")
                        else:
                            print(f"    ✗ Aucun vecteur généré - arrêt passes")
                            break
                    else:
                        print(f"    ✗ Pas assez de points contour ({len(coords)}) - arrêt passes")
                        break
                else:
                    print(f"    ✗ Pas de contour externe - arrêt passes")
                    break
                    
            except Exception as pass_error:
                print(f"    ✗ Erreur passe: {pass_error}")
                break
        
        if concentric_vectors:
            all_vectors.extend(concentric_vectors)
            print(f"\n✓ {len(concentric_vectors)} vecteurs concentriques ajoutés")
        else:
            print(f"\n✗ Aucun vecteur concentrique généré")
        
        # PHASE 4: LIGNES PARALLÈLES
        print(f"\n--- PHASE 4: LIGNES PARALLÈLES ---")
        
        if not current_polygon.is_empty and current_polygon.area > (mower_width_cm * 1.8) ** 2:
            print(f"Zone centrale viable: {current_polygon.area:.1f}cm²")
            
            parallel_vectors = test_generate_parallel_vectors(current_polygon, mower_width_cm)
            
            if parallel_vectors:
                all_vectors.extend(parallel_vectors)
                print(f"✓ {len(parallel_vectors)} vecteurs parallèles ajoutés")
            else:
                print(f"✗ Aucun vecteur parallèle généré")
        else:
            if current_polygon.is_empty:
                print(f"✗ Zone centrale vide")
            else:
                print(f"✗ Zone centrale trop petite: {current_polygon.area:.1f}cm²")
        
    except Exception as e:
        print(f"✗ Erreur globale phase 2-4: {e}")
        import traceback
        traceback.print_exc()
    
    # RÉSULTAT FINAL
    print(f"\n=== RÉSULTAT FINAL ===")
    print(f"Total vecteurs: {len(all_vectors)}")
    
    total_distance = sum(v.get('distance', 0) for v in all_vectors) / 100
    total_rotations = sum(1 for v in all_vectors if v.get('relative_angle', 0) != 0)
    
    print(f"Distance totale: {total_distance:.1f}m")
    print(f"Rotations: {total_rotations}")
    
    # Détail par type
    perimetre_count = len(perimeter_vectors)
    concentrique_count = len(all_vectors) - perimetre_count
    
    print(f"Détail:")
    print(f"  - Périmètre: {perimetre_count} vecteurs")
    print(f"  - Ajouts (concentrique + parallèle): {concentrique_count} vecteurs")
    
    return all_vectors

def test_vectors_to_polygon_points(vectors):
    """Test conversion vecteurs → points"""
    points = [(0, 0)]
    x, y, angle = 0, 0, -90
    
    print(f"  Conversion vecteurs → points:")
    
    for i, vector in enumerate(vectors):
        if vector.get("relative_angle", 0) != 0:
            angle += vector["relative_angle"]
            print(f"    {i}: Rotation {vector['relative_angle']}° → angle total {angle:.1f}°")
        
        distance = vector.get("distance", 0)
        if distance > 0:
            angle_rad = math.radians(angle)
            new_x = x + distance * math.cos(angle_rad)
            new_y = y + distance * math.sin(angle_rad)
            points.append((new_x, new_y))
            print(f"    {i}: Mouvement {distance}cm → point ({new_x:.1f}, {new_y:.1f})")
            x, y = new_x, new_y
    
    return points

def test_contour_to_vectors(coords):
    """Test conversion contour → vecteurs"""
    if len(coords) < 3:
        return []
    
    vectors = []
    current_angle = -90
    
    print(f"    Conversion {len(coords)} coords → vecteurs:")
    
    for i in range(1, len(coords)):
        dx = coords[i][0] - coords[i-1][0]
        dy = coords[i][1] - coords[i-1][1]
        
        if abs(dx) > 0.1 or abs(dy) > 0.1:
            distance = math.sqrt(dx*dx + dy*dy)
            target_angle = math.degrees(math.atan2(dy, dx))
            relative_angle = target_angle - current_angle
            
            # Normaliser angle
            while relative_angle > 180:
                relative_angle -= 360
            while relative_angle <= -180:
                relative_angle += 360
            
            if abs(relative_angle) > 2:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
                print(f"      Rotation: {relative_angle:.1f}°")
            
            vectors.append({"distance": distance, "relative_angle": 0})
            print(f"      Mouvement: {distance:.1f}cm")
            
            current_angle = target_angle
    
    # Fermeture
    dx = coords[0][0] - coords[-1][0]
    dy = coords[0][1] - coords[-1][1]
    distance_to_start = math.sqrt(dx*dx + dy*dy)
    
    if distance_to_start > 5:
        target_angle = math.degrees(math.atan2(dy, dx))
        relative_angle = target_angle - current_angle
        
        while relative_angle > 180:
            relative_angle -= 360
        while relative_angle <= -180:
            relative_angle += 360
        
        if abs(relative_angle) > 2:
            vectors.append({"distance": 0, "relative_angle": relative_angle})
            print(f"      Rotation fermeture: {relative_angle:.1f}°")
        
        vectors.append({"distance": distance_to_start, "relative_angle": 0})
        print(f"      Mouvement fermeture: {distance_to_start:.1f}cm")
    
    return vectors

def test_generate_parallel_vectors(polygon, mower_width):
    """Test génération lignes parallèles"""
    if not SHAPELY_AVAILABLE or polygon.is_empty:
        return []
    
    try:
        minx, miny, maxx, maxy = polygon.bounds
        print(f"  Limites polygone: x[{minx:.1f}, {maxx:.1f}], y[{miny:.1f}, {maxy:.1f}]")
        
        waypoints = []
        y = miny + mower_width / 2
        direction = 1
        line_count = 0
        
        while y < maxy and line_count < 20:  # Limite test
            line = LineString([(minx - mower_width, y), (maxx + mower_width, y)])
            intersection = polygon.intersection(line)
            
            if not intersection.is_empty:
                print(f"    Ligne y={y:.1f}: intersection trouvée")
                
                segments = []
                if hasattr(intersection, 'geoms'):
                    segments = [geom for geom in intersection.geoms if hasattr(geom, 'coords')]
                elif hasattr(intersection, 'coords'):
                    segments = [intersection]
                
                for segment in segments:
                    try:
                        coords = list(segment.coords)
                        if len(coords) >= 2:
                            if direction == 1:
                                waypoints.extend([
                                    {'x': coords[0][0], 'y': coords[0][1]},
                                    {'x': coords[-1][0], 'y': coords[-1][1]}
                                ])
                            else:
                                waypoints.extend([
                                    {'x': coords[-1][0], 'y': coords[-1][1]},
                                    {'x': coords[0][0], 'y': coords[0][1]}
                                ])
                            print(f"      Segment ajouté: {len(coords)} points")
                    except Exception as e:
                        print(f"      Erreur segment: {e}")
            
            y += mower_width * 0.8
            direction *= -1
            line_count += 1
        
        print(f"  ✓ {len(waypoints)} waypoints générés sur {line_count} lignes")
        
        # Convertir en vecteurs
        if waypoints:
            return test_path_to_vectors(waypoints)
        
        return []
        
    except Exception as e:
        print(f"  ✗ Erreur parallèles: {e}")
        return []

def test_path_to_vectors(waypoints):
    """Test conversion waypoints → vecteurs"""
    if len(waypoints) < 2:
        return []
    
    vectors = []
    current_angle = -90
    
    print(f"    Conversion {len(waypoints)} waypoints → vecteurs:")
    
    for i in range(len(waypoints) - 1):
        p1, p2 = waypoints[i], waypoints[i+1]
        dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance > 1.0:
            target_angle = math.degrees(math.atan2(dy, dx))
            relative_angle = target_angle - current_angle
            
            while relative_angle > 180: 
                relative_angle -= 360
            while relative_angle <= -180: 
                relative_angle += 360
            
            if abs(relative_angle) > 2:
                vectors.append({"distance": 0, "relative_angle": relative_angle})
            
            vectors.append({"distance": distance, "relative_angle": 0})
            current_angle = target_angle
    
    print(f"      ✓ {len(vectors)} vecteurs générés")
    return vectors

# SCRIPT PRINCIPAL
if __name__ == "__main__":
    print("=== DIAGNOSTIC GÉNÉRATION PARCOURS AVANCÉ ===")
    print("ATTENTION: Remplacez les vecteurs exemple par les vrais de votre zone_04")
    
    # Récupérer les vrais vecteurs de zone_04
    print("\n1. Pour obtenir les vrais vecteurs de zone_04:")
    print("   - Connectez-vous à votre base de données")
    print("   - Exécutez: SELECT perimeter_vectors FROM zones WHERE name='zone_04'")
    print("   - Remplacez zone_04_vectors dans ce script")
    
    print("\n2. Lancement du test...")
    result = test_zone_04()
    
    print(f"\n3. DIAGNOSTIC:")
    if len(result) <= 10:
        print("   ✗ PROBLÈME: Très peu de vecteurs générés")
        print("   → Vérifiez les logs ci-dessus pour identifier l'étape qui échoue")
    else:
        print("   ✓ Génération semble correcte")
        print("   → Si le problème persiste, c'est dans l'exécution/supervision")
    
    print(f"\n4. À faire ensuite:")
    print("   - Copier ce script sur votre serveur")
    print("   - Remplacer les vecteurs exemple par les vrais")
    print("   - Exécuter et analyser les logs détaillés")