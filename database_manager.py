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

    def delete_zone(self, zone_id):
        """
        Supprime une zone par son ID.
        
        Args:
            zone_id (int): L'ID de la zone à supprimer.
            
        Returns:
            bool: True si supprimé avec succès, False sinon.
        """
        try:
            conn = sqlite3.connect(self.db_name)
            cursor = conn.cursor()
            
            # Vérifier si la zone existe
            cursor.execute("SELECT name FROM zones WHERE id = ?", (zone_id,))
            result = cursor.fetchone()
            
            if not result:
                print(f"Erreur: Aucune zone trouvée avec l'ID {zone_id}.")
                return False
            
            zone_name = result[0]
            
            # Supprimer d'abord l'historique associé (clé étrangère)
            cursor.execute("DELETE FROM mowing_history WHERE zone_id = ?", (zone_id,))
            
            # Supprimer la zone
            cursor.execute("DELETE FROM zones WHERE id = ?", (zone_id,))
            
            conn.commit()
            print(f"Zone '{zone_name}' (ID: {zone_id}) supprimée avec succès.")
            return True
            
        except Exception as e:
            print(f"Erreur lors de la suppression de la zone ID {zone_id}: {e}")
            return False
        finally:
            conn.close()

    def delete_zones_by_ids(self, zone_ids):
        """
        Supprime plusieurs zones par leurs IDs.
        
        Args:
            zone_ids (list): Liste des IDs des zones à supprimer.
            
        Returns:
            dict: Résultat avec succès et échecs détaillés.
        """
        if not zone_ids:
            return {"success": [], "failed": [], "message": "Aucune zone spécifiée."}
        
        success = []
        failed = []
        
        try:
            conn = sqlite3.connect(self.db_name)
            cursor = conn.cursor()
            
            for zone_id in zone_ids:
                try:
                    # Vérifier si la zone existe
                    cursor.execute("SELECT name FROM zones WHERE id = ?", (zone_id,))
                    result = cursor.fetchone()
                    
                    if not result:
                        failed.append({"id": zone_id, "reason": "Zone introuvable"})
                        continue
                    
                    zone_name = result[0]
                    
                    # Supprimer l'historique associé
                    cursor.execute("DELETE FROM mowing_history WHERE zone_id = ?", (zone_id,))
                    
                    # Supprimer la zone
                    cursor.execute("DELETE FROM zones WHERE id = ?", (zone_id,))
                    
                    success.append({"id": zone_id, "name": zone_name})
                    
                except Exception as e:
                    failed.append({"id": zone_id, "reason": str(e)})
            
            conn.commit()
            
        except Exception as e:
            return {"success": [], "failed": zone_ids, "message": f"Erreur de base de données: {e}"}
        finally:
            conn.close()
        
        return {
            "success": success,
            "failed": failed,
            "message": f"{len(success)} zone(s) supprimée(s), {len(failed)} échec(s)"
        }

    def delete_all_zones(self):
        """
        Supprime TOUTES les zones et leur historique.
        Attention: Cette opération est irréversible!
        
        Returns:
            dict: Résultat de l'opération avec statistiques.
        """
        try:
            conn = sqlite3.connect(self.db_name)
            cursor = conn.cursor()
            
            # Compter les zones avant suppression
            cursor.execute("SELECT COUNT(*) FROM zones")
            zones_count = cursor.fetchone()[0]
            
            cursor.execute("SELECT COUNT(*) FROM mowing_history")
            history_count = cursor.fetchone()[0]
            
            if zones_count == 0:
                return {
                    "success": True, 
                    "message": "Aucune zone à supprimer.",
                    "zones_deleted": 0,
                    "history_deleted": 0
                }
            
            # Supprimer tout l'historique
            cursor.execute("DELETE FROM mowing_history")
            
            # Supprimer toutes les zones
            cursor.execute("DELETE FROM zones")
            
            conn.commit()
            
            print(f"Suppression complète: {zones_count} zones et {history_count} entrées d'historique.")
            
            return {
                "success": True,
                "message": f"Toutes les données supprimées avec succès.",
                "zones_deleted": zones_count,
                "history_deleted": history_count
            }
            
        except Exception as e:
            print(f"Erreur lors de la suppression complète: {e}")
            return {
                "success": False,
                "message": f"Erreur: {e}",
                "zones_deleted": 0,
                "history_deleted": 0
            }
        finally:
            conn.close()

    def get_zone_id_by_name(self, name):
        """
        Récupère l'ID d'une zone par son nom.
        
        Args:
            name (str): Le nom de la zone.
            
        Returns:
            int or None: L'ID de la zone ou None si introuvable.
        """
        conn = sqlite3.connect(self.db_name)
        cursor = conn.cursor()
        cursor.execute("SELECT id FROM zones WHERE name = ?", (name,))
        result = cursor.fetchone()
        conn.close()
        
        return result[0] if result else None