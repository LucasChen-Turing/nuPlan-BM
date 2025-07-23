#!/usr/bin/env python3
"""
Script to extract data from a nuPlan database file and export to CSV format.
Extracts ego trajectory, tracked objects, scene info, and traffic light status.
"""

import os
import csv
import sqlite3
from typing import Dict, List, Optional

# Path to the database file
DB_FILE = "data/cache/mini/2021.07.16.18.06.21_veh-38_04933_05307.db"


def execute_many(query_text: str, query_parameters: tuple, db_file: str) -> List[sqlite3.Row]:
    """
    Runs a query with the provided arguments on a specified Sqlite DB file.
    This query can return any number of rows.
    """
    connection = sqlite3.connect(db_file)
    connection.row_factory = sqlite3.Row
    cursor = connection.cursor()

    try:
        cursor.execute(query_text, query_parameters)
        return cursor.fetchall()
    finally:
        cursor.close()
        connection.close()


def export_ego_trajectory_to_csv(db_file: str, output_file: str) -> None:
    """
    Extract ego vehicle trajectory and export to CSV.
    """
    print(f"Extracting ego trajectory to {output_file}...")
    
    query = """
    SELECT ep.timestamp, ep.x, ep.y, ep.z, ep.vx, ep.vy, ep.vz,
           ep.qw, ep.qx, ep.qy, ep.qz
    FROM ego_pose AS ep
    INNER JOIN lidar_pc AS lp ON lp.ego_pose_token = ep.token
    ORDER BY ep.timestamp;
    """
    
    results = execute_many(query, (), db_file)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz', 'qw', 'qx', 'qy', 'qz']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow({
                'timestamp': row['timestamp'],
                'x': row['x'],
                'y': row['y'],
                'z': row['z'],
                'vx': row['vx'],
                'vy': row['vy'],
                'vz': row['vz'],
                'qw': row['qw'],
                'qx': row['qx'],
                'qy': row['qy'],
                'qz': row['qz']
            })
    
    print(f"Exported {len(results)} ego trajectory records to {output_file}")


def export_tracked_objects_to_csv(db_file: str, output_file: str) -> None:
    """
    Extract all tracked objects and export to CSV.
    """
    print(f"Extracting tracked objects to {output_file}...")
    
    query = """
    SELECT lp.timestamp, c.name AS category_name, 
           lb.x, lb.y, lb.z, lb.yaw, 
           lb.width, lb.length, lb.height,
           lb.vx, lb.vy, lb.track_token
    FROM lidar_box AS lb
    INNER JOIN track AS t ON t.token = lb.track_token
    INNER JOIN category AS c ON c.token = t.category_token
    INNER JOIN lidar_pc AS lp ON lp.token = lb.lidar_pc_token
    ORDER BY lp.timestamp, lb.track_token;
    """
    
    results = execute_many(query, (), db_file)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'category_name', 'x', 'y', 'z', 'yaw', 
                     'width', 'length', 'height', 'vx', 'vy', 'track_token']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow({
                'timestamp': row['timestamp'],
                'category_name': row['category_name'],
                'x': row['x'],
                'y': row['y'],
                'z': row['z'],
                'yaw': row['yaw'],
                'width': row['width'],
                'length': row['length'],
                'height': row['height'],
                'vx': row['vx'],
                'vy': row['vy'],
                'track_token': row['track_token'].hex()
            })
    
    print(f"Exported {len(results)} tracked object records to {output_file}")


def export_lidar_metadata_to_csv(db_file: str, output_file: str) -> None:
    """
    Extract LiDAR point cloud metadata and export to CSV.
    """
    print(f"Extracting LiDAR metadata to {output_file}...")
    
    query = """
    SELECT lp.token, lp.filename, lp.timestamp,
           ep.x AS ego_x, ep.y AS ego_y, ep.z AS ego_z
    FROM lidar_pc AS lp
    INNER JOIN ego_pose AS ep ON lp.ego_pose_token = ep.token
    ORDER BY lp.timestamp;
    """
    
    results = execute_many(query, (), db_file)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['token', 'filename', 'timestamp', 'ego_x', 'ego_y', 'ego_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow({
                'token': row['token'].hex(),
                'filename': row['filename'],
                'timestamp': row['timestamp'],
                'ego_x': row['ego_x'],
                'ego_y': row['ego_y'],
                'ego_z': row['ego_z']
            })
    
    print(f"Exported {len(results)} LiDAR metadata records to {output_file}")


def export_scenes_to_csv(db_file: str, output_file: str) -> None:
    """
    Extract scene information and export to CSV.
    """
    print(f"Extracting scenes to {output_file}...")
    
    query = """
    SELECT s.token, s.name, s.roadblock_ids, 
           ep.x AS goal_x, ep.y AS goal_y, ep.z AS goal_z
    FROM scene AS s
    LEFT JOIN ego_pose AS ep ON s.goal_ego_pose_token = ep.token;
    """
    
    results = execute_many(query, (), db_file)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['token', 'name', 'roadblock_ids', 'goal_x', 'goal_y', 'goal_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow({
                'token': row['token'].hex(),
                'name': row['name'],
                'roadblock_ids': row['roadblock_ids'],
                'goal_x': row['goal_x'],
                'goal_y': row['goal_y'],
                'goal_z': row['goal_z']
            })
    
    print(f"Exported {len(results)} scene records to {output_file}")


def export_traffic_lights_to_csv(db_file: str, output_file: str) -> None:
    """
    Extract traffic light status and export to CSV.
    """
    print(f"Extracting traffic light status to {output_file}...")
    
    query = """
    SELECT tls.lane_connector_id, tls.status, lp.timestamp
    FROM traffic_light_status AS tls
    INNER JOIN lidar_pc AS lp ON tls.lidar_pc_token = lp.token
    ORDER BY lp.timestamp, tls.lane_connector_id;
    """
    
    results = execute_many(query, (), db_file)
    
    with open(output_file, 'w', newline='') as csvfile:
        fieldnames = ['timestamp', 'lane_connector_id', 'status']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        
        writer.writeheader()
        for row in results:
            writer.writerow({
                'timestamp': row['timestamp'],
                'lane_connector_id': row['lane_connector_id'],
                'status': row['status']
            })
    
    print(f"Exported {len(results)} traffic light records to {output_file}")


def main():
    """
    Main function to extract all data types from the database to CSV files.
    """
    # Ensure the database file exists
    if not os.path.exists(DB_FILE):
        print(f"Error: Database file {DB_FILE} not found!")
        return
    
    # Create output directory
    output_dir = "cache/mini/csv_exports"
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract base filename for output naming
    base_name = os.path.splitext(os.path.basename(DB_FILE))[0]
    
    print(f"Extracting data from: {DB_FILE}")
    print(f"Output directory: {output_dir}")
    print("=" * 50)
    
    # Export all data types
    export_ego_trajectory_to_csv(DB_FILE, f"{output_dir}/{base_name}_ego_trajectory.csv")
    export_tracked_objects_to_csv(DB_FILE, f"{output_dir}/{base_name}_tracked_objects.csv")
    export_lidar_metadata_to_csv(DB_FILE, f"{output_dir}/{base_name}_lidar_metadata.csv")
    export_scenes_to_csv(DB_FILE, f"{output_dir}/{base_name}_scenes.csv")
    export_traffic_lights_to_csv(DB_FILE, f"{output_dir}/{base_name}_traffic_lights.csv")
    
    print("=" * 50)
    print("Data extraction completed!")


if __name__ == "__main__":
    main()