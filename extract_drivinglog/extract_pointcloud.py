#!/usr/bin/env python3
"""
Script to extract point cloud data from a nuPlan database file.
"""

import os
import sqlite3
import numpy as np
from pathlib import Path
from typing import Optional, Tuple

# Path to the database file
DB_FILE = "data/cache/mini/2021.07.16.20.45.29_veh-35_01095_01486.db"


def execute_one(query_text: str, query_parameters: tuple, db_file: str) -> Optional[sqlite3.Row]:
    """
    Runs a query with the provided arguments on a specified Sqlite DB file.
    Validates that the query returns at most one row.
    """
    connection = sqlite3.connect(db_file)
    connection.row_factory = sqlite3.Row
    cursor = connection.cursor()

    try:
        cursor.execute(query_text, query_parameters)
        result = cursor.fetchone()
        
        if result is not None and cursor.fetchone() is not None:
            raise RuntimeError("execute_one query returned multiple rows.")
        
        return result
    finally:
        cursor.close()
        connection.close()


def get_lidar_pc_filename(db_file: str, lidar_pc_token: str) -> Tuple[str, int]:
    """
    Get the filename and timestamp for a lidar point cloud.
    
    Args:
        db_file: Path to the database file
        lidar_pc_token: Hexadecimal token for the lidar point cloud
        
    Returns:
        Tuple of (filename, timestamp)
    """
    query = """
    SELECT lp.filename, lp.timestamp
    FROM lidar_pc AS lp
    WHERE lp.token = ?
    """
    
    result = execute_one(query, (bytearray.fromhex(lidar_pc_token),), db_file)
    if result is None:
        raise ValueError(f"No lidar_pc found with token {lidar_pc_token}")
    
    return result['filename'], result['timestamp']


def load_point_cloud(db_file: str, lidar_pc_token: str, output_file: str = None) -> np.ndarray:
    """
    Load a point cloud from the database.
    
    Args:
        db_file: Path to the database file
        lidar_pc_token: Hexadecimal token for the lidar point cloud
        output_file: Optional path to save the point cloud as .npy file
        
    Returns:
        Numpy array of shape (N, 4) containing x, y, z, intensity
    """
    # Get the filename for the point cloud
    filename, timestamp = get_lidar_pc_filename(db_file, lidar_pc_token)
    
    # Construct the path to the point cloud file
    data_root = Path(db_file).parent
    log_name = Path(db_file).name.split('.db')[0]
    point_cloud_path = data_root / "sensor_blobs" / log_name / "lidar" / filename
    
    print(f"Looking for point cloud file: {point_cloud_path}")
    
    # Check if the file exists
    if not point_cloud_path.exists():
        print(f"Point cloud file not found at {point_cloud_path}")
        print("Note: The sensor_blobs directory might not be available in the mini dataset.")
        print("This script requires the full dataset with sensor data.")
        return None
    
    # Load the point cloud
    try:
        # Point clouds are typically stored as binary files with x, y, z, intensity values
        # The exact format may vary, but a common format is 4 float32 values per point
        with open(point_cloud_path, 'rb') as f:
            data = np.frombuffer(f.read(), dtype=np.float32)
            # Reshape to (N, 4) for x, y, z, intensity
            point_cloud = data.reshape(-1, 4)
        
        print(f"Loaded point cloud with {len(point_cloud)} points")
        
        # Save to output file if specified
        if output_file:
            np.save(output_file, point_cloud)
            print(f"Saved point cloud to {output_file}")
        
        return point_cloud
    except Exception as e:
        print(f"Error loading point cloud: {e}")
        return None


def get_sample_lidar_pc_token(db_file: str) -> str:
    """
    Get a sample lidar point cloud token from the database.
    """
    query = "SELECT token FROM lidar_pc LIMIT 1;"
    result = execute_one(query, (), db_file)
    return result['token'].hex()


def main():
    """
    Main function to extract a point cloud from the database.
    """
    # Ensure the database file exists
    if not os.path.exists(DB_FILE):
        print(f"Error: Database file {DB_FILE} not found!")
        return
    
    # Get a sample lidar point cloud token
    sample_token = get_sample_lidar_pc_token(DB_FILE)
    print(f"Using lidar_pc token: {sample_token}")
    
    # Extract the point cloud
    output_file = "point_cloud.npy"
    point_cloud = load_point_cloud(DB_FILE, sample_token, output_file)
    
    if point_cloud is not None:
        print(f"Point cloud shape: {point_cloud.shape}")
        print(f"Point cloud min values: {point_cloud.min(axis=0)}")
        print(f"Point cloud max values: {point_cloud.max(axis=0)}")


if __name__ == "__main__":
    main() 