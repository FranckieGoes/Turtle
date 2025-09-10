# -*- coding: utf-8 -*-
import sqlite3
import json
import time

class DatabaseManager:
    """
    Classe pour gérer les interactions avec la base de données SQLite.
    """
    def __init__(self, db_name="mowing_zones.db"):
        self.db_name = db_name
        self._create_tables()

    def _create_tables(self):
        """Crée les tables nécessaires si elles n'existent pas."""
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        # Table pour stocker les zones de tonte
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS zones (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                name TEXT NOT NULL UNIQUE,
                perimeter_vectors TEXT NOT NULL,
                created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        
        # Table pour stocker l'historique des tontes
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS mowing_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                zone_id INTEGER,
                start_time TIMESTAMP NOT NULL,
                end_time TIMESTAMP,
                routes TEXT,
                duration_minutes REAL,
                energy_used_wh REAL,
                obstacles_encountered TEXT,
                FOREIGN KEY (zone_id) REFERENCES zones (id)
            )
        """)
        
        conn.commit()
        conn.close()

    def save_zone(self, name, perimeter_vectors):
        """
        Sauvegarde un nouveau périmètre de zone de tonte.
        
        Args:
            name (str): Le nom de la zone.
            perimeter_vectors (list): Une liste de dictionnaires représentant les segments.
        """
        try:
            conn = sqlite3.connect(self.db_name)
            cursor = conn.cursor()
            
            # Sérialiser les vecteurs en une chaîne JSON
            vectors_json = json.dumps(perimeter_vectors)
            
            cursor.execute("INSERT INTO zones (name, perimeter_vectors) VALUES (?, ?)", (name, vectors_json))
            conn.commit()
            print(f"Zone '{name}' sauvegardée avec succès.")
            return True
        except sqlite3.IntegrityError:
            print(f"Erreur: Une zone avec le nom '{name}' existe déjà.")
            return False
        finally:
            conn.close()

    def get_zone_by_name(self, name):
        """
        Récupère les vecteurs d'un périmètre par son nom.
        
        Returns:
            list or None: La liste des vecteurs ou None si la zone n'existe pas.
        """
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("SELECT perimeter_vectors FROM zones WHERE name = ?", (name,))
        result = cursor.fetchone()
        conn.close()
        
        if result:
            return json.loads(result[0])
        return None

    def get_all_zones(self):
        """
        Récupère tous les noms et IDs de zones de tonte.
        
        Returns:
            list: Une liste de dictionnaires avec l'id et le nom de chaque zone.
        """
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("SELECT id, name FROM zones")
        results = [{"id": row[0], "name": row[1]} for row in cursor.fetchall()]
        conn.close()
        return results

    def save_mowing_history(self, zone_id, routes, duration_minutes, energy_used_wh, obstacles_encountered):
        """
        Enregistre l'historique d'une tonte.
        
        Args:
            zone_id (int): L'ID de la zone tondue.
            routes (list): Une liste de vecteurs représentant le chemin parcouru.
            duration_minutes (float): La durée de la tonte en minutes.
            energy_used_wh (float): L'énergie consommée en wattheures.
            obstacles_encountered (list): Une liste d'obstacles rencontrés.
        """
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        
        routes_json = json.dumps(routes)
        obstacles_json = json.dumps(obstacles_encountered)
        
        cursor.execute("""
            INSERT INTO mowing_history (zone_id, start_time, routes, duration_minutes, energy_used_wh, obstacles_encountered)
            VALUES (?, ?, ?, ?, ?, ?)
        """, (zone_id, int(time.time()), routes_json, duration_minutes, energy_used_wh, obstacles_json))
        
        conn.commit()
        conn.close()
        print(f"Historique de tonte sauvegardé pour la zone ID {zone_id}.")
