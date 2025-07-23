#!/usr/bin/env python3
"""
Example script to extract data from a nuPlan database file.
This script demonstrates how to access data from the .db files using direct SQL queries.
"""

import os
from pathlib import Path
import sqlite3
from typing import Dict, List, Optional, Set, Tuple

# Path to the database file
DB_FILE = "data/cache/mini/2021.07.16.20.45.29_veh-35_01095_01486.db"


def execute_one(query_text: str, query_parameters: tuple, db_file: str) -> Optional[sqlite3.Row]:
    """
    Runs a query with the provided arguments on a specified Sqlite DB file.
    Validates that the query returns at most one row.
    :param query_text: The query to run.
    :param query_parameters: The parameters to provide to the query.
    :param db_file: The DB file on which to run the query.
    :return: The returned row, if it exists. None otherwise.
    """
    connection = sqlite3.connect(db_file)
    connection.row_factory = sqlite3.Row
    cursor = connection.cursor()

    try:
        cursor.execute(query_text, query_parameters)
        result = cursor.fetchone()
        
        # Check for more rows. If more exist, throw an error.
        if result is not None and cursor.fetchone() is not None:
            raise RuntimeError("execute_one query returned multiple rows.")
        
        return result
    finally:
        cursor.close()
        connection.close()


def execute_many(query_text: str, query_parameters: tuple, db_file: str) -> List[sqlite3.Row]:
    """
    Runs a query with the provided arguments on a specified Sqlite DB file.
    This query can return any number of rows.
    :param query_text: The query to run.
    :param query_parameters: The parameters to provide to the query.
    :param db_file: The DB file on which to run the query.
    :return: A list of rows emitted from the query.
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


def print_db_info(db_file: str) -> None:
    """
    Print basic information about the database.
    """
    print(f"\n=== Database Info: {os.path.basename(db_file)} ===")
    
    # Get the number of lidar point clouds
    query = "SELECT COUNT(*) AS cnt FROM lidar_pc;"
    result = execute_one(query, (), db_file)
    print(f"Number of lidar point clouds: {result['cnt']}")
    
    # Get the number of scenes
    query = "SELECT COUNT(*) AS cnt FROM scene;"
    result = execute_one(query, (), db_file)
    print(f"Number of scenes: {result['cnt']}")
    
    # Get the number of tracked objects
    query = "SELECT COUNT(*) AS cnt FROM lidar_box;"
    result = execute_one(query, (), db_file)
    print(f"Number of tracked objects (lidar boxes): {result['cnt']}")
    
    # Get the log information
    query = "SELECT * FROM log;"
    result = execute_one(query, (), db_file)
    print(f"Log info: Vehicle: {result['vehicle_name']}, Date: {result['date']}, Location: {result['location']}")
    
    # Get the duration of the log
    query = """
    SELECT MIN(timestamp) as min_time, MAX(timestamp) as max_time 
    FROM lidar_pc;
    """
    result = execute_one(query, (), db_file)
    duration_sec = (result['max_time'] - result['min_time']) / 1e6  # Convert microseconds to seconds
    print(f"Log duration: {duration_sec:.2f} seconds")


def get_sample_lidar_pc_token(db_file: str) -> str:
    """
    Get a sample lidar point cloud token from the database.
    """
    query = "SELECT token FROM lidar_pc LIMIT 1;"
    result = execute_one(query, (), db_file)
    return result['token'].hex()


def extract_ego_trajectory(db_file: str, limit: int = 10) -> None:
    """
    Extract and print a portion of the ego vehicle trajectory.
    """
    print(f"\n=== Ego Vehicle Trajectory (first {limit} samples) ===")
    
    query = """
    SELECT ep.token, ep.timestamp, ep.x, ep.y, ep.z, ep.vx, ep.vy, ep.vz
    FROM ego_pose AS ep
    INNER JOIN lidar_pc AS lp ON lp.ego_pose_token = ep.token
    ORDER BY ep.timestamp
    LIMIT ?;
    """
    
    results = execute_many(query, (limit,), db_file)
    for i, row in enumerate(results):
        print(f"Sample {i}: Position: ({row['x']:.2f}, {row['y']:.2f}, {row['z']:.2f}), "
              f"Velocity: ({row['vx']:.2f}, {row['vy']:.2f}, {row['vz']:.2f}), "
              f"Timestamp: {row['timestamp']}")


def extract_tracked_objects(db_file: str, lidar_pc_token: str) -> None:
    """
    Extract tracked objects for a specific lidar point cloud.
    """
    print(f"\n=== Tracked Objects for LidarPC token: {lidar_pc_token} ===")
    
    query = """
    SELECT c.name AS category_name, 
           lb.x, lb.y, lb.z, lb.yaw, 
           lb.width, lb.length, lb.height,
           lb.vx, lb.vy, lb.track_token
    FROM lidar_box AS lb
    INNER JOIN track AS t ON t.token = lb.track_token
    INNER JOIN category AS c ON c.token = t.category_token
    INNER JOIN lidar_pc AS lp ON lp.token = lb.lidar_pc_token
    WHERE lp.token = ?
    LIMIT 10;
    """
    
    results = execute_many(query, (bytearray.fromhex(lidar_pc_token),), db_file)
    
    for i, row in enumerate(results):
        print(f"Object {i}: Category: {row['category_name']}, "
              f"Position: ({row['x']:.2f}, {row['y']:.2f}, {row['z']:.2f}), "
              f"Size: {row['width']:.2f}x{row['length']:.2f}x{row['height']:.2f}, "
              f"Velocity: ({row['vx']:.2f}, {row['vy']:.2f}), "
              f"Track token: {row['track_token'].hex()}")


def extract_scene_info(db_file: str) -> None:
    """
    Extract information about scenes in the database.
    """
    print("\n=== Scene Information ===")
    
    query = """
    SELECT s.token, s.name, s.roadblock_ids, ep.x, ep.y, ep.z
    FROM scene AS s
    LEFT JOIN ego_pose AS ep ON s.goal_ego_pose_token = ep.token
    LIMIT 5;
    """
    
    results = execute_many(query, (), db_file)
    
    for i, row in enumerate(results):
        goal_pos = "None" if row['x'] is None else f"({row['x']:.2f}, {row['y']:.2f}, {row['z']:.2f})"
        print(f"Scene {i}: Name: {row['name']}, Goal position: {goal_pos}")
        if row['roadblock_ids']:
            print(f"  Roadblock IDs: {row['roadblock_ids']}")


def extract_traffic_light_status(db_file: str, limit: int = 5) -> None:
    """
    Extract traffic light status information.
    """
    print("\n=== Traffic Light Status ===")
    
    query = """
    SELECT tls.lane_connector_id, tls.status, lp.timestamp
    FROM traffic_light_status AS tls
    INNER JOIN lidar_pc AS lp ON tls.lidar_pc_token = lp.token
    ORDER BY lp.timestamp
    LIMIT ?;
    """
    
    results = execute_many(query, (limit,), db_file)
    
    for i, row in enumerate(results):
        print(f"Traffic Light {i}: Lane connector ID: {row['lane_connector_id']}, "
              f"Status: {row['status']}, Timestamp: {row['timestamp']}")


def main():
    """
    Main function to demonstrate different ways to extract data from a nuPlan database.
    """
    # Ensure the database file exists
    if not os.path.exists(DB_FILE):
        print(f"Error: Database file {DB_FILE} not found!")
        return
    
    # Print basic database information
    print_db_info(DB_FILE)
    
    # Get a sample lidar point cloud token
    sample_token = get_sample_lidar_pc_token(DB_FILE)
    
    # Extract ego trajectory
    extract_ego_trajectory(DB_FILE)
    
    # Extract tracked objects
    extract_tracked_objects(DB_FILE, sample_token)
    
    # Extract scene information
    extract_scene_info(DB_FILE)
    
    # Extract traffic light status
    extract_traffic_light_status(DB_FILE)


if __name__ == "__main__":
    main() 