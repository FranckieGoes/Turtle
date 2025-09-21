# Fichier: lidar_ld06_manager.py

import serial
import time
import threading
import struct
import numpy as np
from collections import deque
import math

class LD06LidarManager:
    """
    Gestionnaire pour le LIDAR LD06 connecté en USB
    Compatible avec le système robotique existant
    """
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=230400):
        """
        Initialise le gestionnaire LIDAR LD06
        
        Args:
            port (str): Port série du LIDAR
            baudrate (int): Vitesse de communication (230400 par défaut pour LD06)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial_connection = None
        
        # Threading
        self.is_running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        
        # Données LIDAR
        self.current_scan = {}  # {angle: distance}
        self.last_full_scan = {}
        self.scan_history = deque(maxlen=10)  # Garder les 10 derniers scans
        
        # Statistiques
        self.total_points = 0
        self.scan_count = 0
        
        # Callbacks pour intégration robot
        self.obstacle_callback = None
        self.scan_complete_callback = None
        
    def connect(self):
        """Établit la connexion avec le LIDAR"""
        try:
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0
            )
            print(f"LIDAR LD06 connecté sur {self.port}")
            return True
        except Exception as e:
            print(f"Erreur de connexion LIDAR: {e}")
            return False
    
    def start_scanning(self):
        """Démarre l'acquisition des données LIDAR"""
        if not self.serial_connection or not self.serial_connection.is_open:
            if not self.connect():
                return False
        
        self.is_running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        print("Acquisition LIDAR démarrée")
        return True
    
    def stop_scanning(self):
        """Arrête l'acquisition des données LIDAR"""
        self.is_running = False
        if self.read_thread:
            self.read_thread.join(timeout=2.0)
        if self.serial_connection:
            self.serial_connection.close()
        print("Acquisition LIDAR arrêtée")
    
    def _read_loop(self):
        """Boucle de lecture des données LIDAR (thread séparé)"""
        buffer = b''
        
        while self.is_running:
            try:
                # Lire les données disponibles
                if self.serial_connection.in_waiting:
                    data = self.serial_connection.read(self.serial_connection.in_waiting)
                    buffer += data
                    
                    # Traiter les paquets complets
                    while len(buffer) >= 47:  # Taille d'un paquet LD06
                        if buffer[0] == 0x54:  # Header du paquet LD06
                            packet = buffer[:47]
                            buffer = buffer[47:]
                            self._process_packet(packet)
                        else:
                            buffer = buffer[1:]  # Ignorer le byte et chercher le prochain header
                else:
                    time.sleep(0.001)  # Pause courte si pas de données
                    
            except Exception as e:
                if self.is_running:
                    print(f"Erreur lecture LIDAR: {e}")
                    time.sleep(0.1)
    
    def _process_packet(self, packet):
        """Traite un paquet de données LD06"""
        try:
            if len(packet) < 47 or packet[0] != 0x54:
                return
            
            # Décodage du paquet LD06
            header = packet[0]
            ver_len = packet[1]
            speed = struct.unpack('<H', packet[2:4])[0]  # Vitesse de rotation
            start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0  # Angle de départ
            
            # 12 mesures par paquet
            points = []
            for i in range(12):
                offset = 6 + i * 3
                distance = struct.unpack('<H', packet[offset:offset+2])[0]
                confidence = packet[offset+2]
                
                angle = (start_angle + i * 0.83) % 360  # Incrément d'environ 0.83° par mesure
                
                if distance > 0 and confidence > 15:  # Filtrer les mesures de faible qualité
                    points.append({
                        'angle': angle,
                        'distance': distance,  # en mm
                        'confidence': confidence
                    })
            
            # Mettre à jour les données
            with self.data_lock:
                for point in points:
                    self.current_scan[point['angle']] = point
                    self.total_points += 1
                
                # Détecter un scan complet (retour proche de 0°)
                if points and any(p['angle'] < 10 for p in points) and len(self.current_scan) > 200:
                    self.last_full_scan = self.current_scan.copy()
                    self.scan_history.append(self.last_full_scan.copy())
                    self.current_scan.clear()
                    self.scan_count += 1
                    
                    # Callback pour scan complet
                    if self.scan_complete_callback:
                        self.scan_complete_callback(self.last_full_scan)
                    
                    # Détection d'obstacles
                    self._check_obstacles()
                        
        except Exception as e:
            print(f"Erreur traitement paquet: {e}")
    
    def _check_obstacles(self):
        """Vérifie la présence d'obstacles dans le scan"""
        if not self.last_full_scan or not self.obstacle_callback:
            return
        
        # Zone de sécurité (secteur devant le robot)
        front_angles = [(a, d['distance']) for a, d in self.last_full_scan.items() 
                       if 340 <= a <= 360 or 0 <= a <= 20]
        
        if front_angles:
            min_distance = min(d for _, d in front_angles)
            if min_distance < 300:  # Obstacle à moins de 30cm
                self.obstacle_callback(min_distance, front_angles)
    
    def get_current_scan(self):
        """Retourne le dernier scan complet"""
        with self.data_lock:
            return self.last_full_scan.copy()
    
    def get_distance_at_angle(self, target_angle, tolerance=5):
        """
        Retourne la distance mesurée à un angle donné
        
        Args:
            target_angle (float): Angle recherché (0-360°)
            tolerance (float): Tolérance angulaire
        
        Returns:
            float: Distance en mm, ou None si pas de mesure
        """
        with self.data_lock:
            for angle, point in self.last_full_scan.items():
                if abs(angle - target_angle) <= tolerance:
                    return point['distance']
        return None
    
    def get_front_distance(self):
        """Retourne la distance minimale devant le robot (±15°)"""
        distances = []
        with self.data_lock:
            for angle, point in self.last_full_scan.items():
                if (345 <= angle <= 360) or (0 <= angle <= 15):
                    distances.append(point['distance'])
        
        return min(distances) if distances else None
    
    def set_obstacle_callback(self, callback):
        """Définit la fonction de callback pour les obstacles détectés"""
        self.obstacle_callback = callback
    
    def set_scan_callback(self, callback):
        """Définit la fonction de callback pour chaque scan complet"""
        self.scan_complete_callback = callback
    
    def get_statistics(self):
        """Retourne les statistiques d'acquisition"""
        return {
            'total_points': self.total_points,
            'scan_count': self.scan_count,
            'points_per_scan': self.total_points / max(1, self.scan_count),
            'is_running': self.is_running,
            'last_scan_size': len(self.last_full_scan)
        }
    
    def cleanup(self):
        """Nettoyage des ressources"""
        self.stop_scanning()


# Exemple d'intégration avec ton robot
class RobotWithLidar:
    """Exemple d'intégration du LIDAR avec ton RobotController"""
    
    def __init__(self, robot_controller):
        self.robot = robot_controller
        self.lidar = LD06LidarManager()
        
        # Configuration des callbacks
        self.lidar.set_obstacle_callback(self.on_obstacle_detected)
        self.lidar.set_scan_callback(self.on_scan_complete)
        
        # États
        self.autonomous_mode = False
        self.obstacle_avoidance_enabled = True
        
    def start_lidar(self):
        """Démarre le LIDAR"""
        return self.lidar.start_scanning()
    
    def stop_lidar(self):
        """Arrête le LIDAR"""
        self.lidar.stop_scanning()
    
    def on_obstacle_detected(self, distance, angles_data):
        """Callback appelé quand un obstacle est détecté"""
        print(f"⚠️ Obstacle détecté à {distance}mm")
        
        if self.obstacle_avoidance_enabled and self.autonomous_mode:
            # Arrêter le mouvement actuel
            self.robot.stop()
            
            # Logique d'évitement (à adapter selon tes besoins)
            self.avoid_obstacle(distance)
    
    def on_scan_complete(self, scan_data):
        """Callback appelé à chaque scan complet"""
        print(f"Scan complet: {len(scan_data)} points")
        # Ici tu peux faire de la cartographie, navigation, etc.
    
    def avoid_obstacle(self, obstacle_distance):
        """Logique d'évitement d'obstacle"""
        print("Évitement d'obstacle en cours...")
        
        # Exemple simple : tourner à droite
        self.robot.select_type_rotate(45, 30, "right", 2.0)
        time.sleep(2)  # Attendre la fin du virage
        
        # Avancer un peu
        self.robot.move_for_distance(2.0, 50)  # 50cm à 2 km/h
    
    def get_lidar_stats(self):
        """Retourne les statistiques du LIDAR"""
        return self.lidar.get_statistics()
    
    def cleanup(self):
        """Nettoyage complet"""
        self.lidar.cleanup()
        self.robot.cleanup()


# Script de test
if __name__ == "__main__":
    # Test basique du LIDAR
    lidar = LD06LidarManager()
    
    if lidar.start_scanning():
        print("LIDAR démarré, acquisition en cours...")
        
        try:
            time.sleep(10)  # Acquérir pendant 10 secondes
            
            # Afficher quelques mesures
            scan = lidar.get_current_scan()
            print(f"Scan actuel: {len(scan)} points")
            
            front_dist = lidar.get_front_distance()
            if front_dist:
                print(f"Distance devant: {front_dist}mm")
            
            # Statistiques
            stats = lidar.get_statistics()
            print(f"Statistiques: {stats}")
            
        except KeyboardInterrupt:
            print("Arrêt demandé par l'utilisateur")
        finally:
            lidar.cleanup()
    else:
        print("Impossible de démarrer le LIDAR")