#!/usr/bin/env python3
"""
Script to extract camera information from a nuPlan database file.
"""

import os
import sqlite3
import numpy as np
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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


def get_camera_info(db_file: str) -> None:
    """
    Extract and print camera information from the database.
    """
    print("\n=== Camera Information ===")
    
    query = """
    SELECT c.token, c.channel, c.model, c.width, c.height
    FROM camera AS c
    """
    
    results = execute_many(query, (), db_file)
    
    for i, row in enumerate(results):
        print(f"Camera {i+1}: Channel: {row['channel']}, Model: {row['model']}, "
              f"Resolution: {row['width']}x{row['height']}")


def get_camera_calibration(db_file: str, camera_token: str) -> None:
    """
    Extract and print camera calibration parameters.
    """
    print(f"\n=== Camera Calibration for token: {camera_token} ===")
    
    query = """
    SELECT c.channel, c.translation, c.rotation, c.intrinsic, c.distortion
    FROM camera AS c
    WHERE c.token = ?
    """
    
    result = execute_one(query, (bytearray.fromhex(camera_token),), db_file)
    
    if result is None:
        print(f"No camera found with token {camera_token}")
        return
    
    print(f"Channel: {result['channel']}")
    
    # Camera parameters are stored as binary blobs
    # We need to convert them to numpy arrays
    # Handle each parameter separately with error handling
    
    # Translation
    try:
        if result['translation'] is not None and len(result['translation']) > 0:
            translation = np.frombuffer(result['translation'], dtype=np.float64)
            print(f"Translation (x, y, z): {translation}")
        else:
            print("Translation: Not available")
    except Exception as e:
        print(f"Error parsing translation: {e}")
        print(f"Translation binary data length: {len(result['translation']) if result['translation'] is not None else 0}")
    
    # Rotation
    try:
        if result['rotation'] is not None and len(result['rotation']) > 0:
            rotation = np.frombuffer(result['rotation'], dtype=np.float64)
            print(f"Rotation (quaternion): {rotation}")
        else:
            print("Rotation: Not available")
    except Exception as e:
        print(f"Error parsing rotation: {e}")
        print(f"Rotation binary data length: {len(result['rotation']) if result['rotation'] is not None else 0}")
    
    # Intrinsic
    try:
        if result['intrinsic'] is not None and len(result['intrinsic']) > 0:
            # Check if the data size is appropriate for a 3x3 matrix (9 float64 values)
            if len(result['intrinsic']) == 9 * 8:  # 9 float64 values (8 bytes each)
                intrinsic = np.frombuffer(result['intrinsic'], dtype=np.float64).reshape(3, 3)
                print(f"Intrinsic matrix:\n{intrinsic}")
            else:
                print(f"Intrinsic matrix: Invalid size for 3x3 matrix ({len(result['intrinsic'])} bytes)")
        else:
            print("Intrinsic matrix: Not available")
    except Exception as e:
        print(f"Error parsing intrinsic matrix: {e}")
        print(f"Intrinsic binary data length: {len(result['intrinsic']) if result['intrinsic'] is not None else 0}")
    
    # Distortion
    try:
        if result['distortion'] is not None and len(result['distortion']) > 0:
            distortion = np.frombuffer(result['distortion'], dtype=np.float64)
            print(f"Distortion coefficients: {distortion}")
        else:
            print("Distortion coefficients: Not available")
    except Exception as e:
        print(f"Error parsing distortion coefficients: {e}")
        print(f"Distortion binary data length: {len(result['distortion']) if result['distortion'] is not None else 0}")
    
    # Print raw binary data lengths for debugging
    print("\nRaw binary data lengths:")
    print(f"Translation: {len(result['translation']) if result['translation'] is not None else 0} bytes")
    print(f"Rotation: {len(result['rotation']) if result['rotation'] is not None else 0} bytes")
    print(f"Intrinsic: {len(result['intrinsic']) if result['intrinsic'] is not None else 0} bytes")
    print(f"Distortion: {len(result['distortion']) if result['distortion'] is not None else 0} bytes")


def get_image_info(db_file: str, limit: int = 5) -> None:
    """
    Extract and print information about images in the database.
    """
    print(f"\n=== Image Information (first {limit} images) ===")
    
    query = """
    SELECT i.token, i.filename_jpg, i.timestamp, c.channel
    FROM image AS i
    INNER JOIN camera AS c ON i.camera_token = c.token
    ORDER BY i.timestamp
    LIMIT ?
    """
    
    results = execute_many(query, (limit,), db_file)
    
    for i, row in enumerate(results):
        print(f"Image {i+1}: Channel: {row['channel']}, "
              f"Filename: {row['filename_jpg']}, "
              f"Timestamp: {row['timestamp']}")


def get_camera_token_by_channel(db_file: str, channel: str) -> Optional[str]:
    """
    Get the token for a camera with the specified channel.
    """
    query = """
    SELECT c.token
    FROM camera AS c
    WHERE c.channel = ?
    """
    
    result = execute_one(query, (channel,), db_file)
    
    if result is None:
        return None
    
    return result['token'].hex()


def get_image_count_by_camera(db_file: str) -> None:
    """
    Count the number of images for each camera.
    """
    print("\n=== Image Count by Camera ===")
    
    query = """
    SELECT c.channel, COUNT(i.token) as image_count
    FROM camera AS c
    LEFT JOIN image AS i ON i.camera_token = c.token
    GROUP BY c.channel
    ORDER BY image_count DESC
    """
    
    results = execute_many(query, (), db_file)
    
    for row in results:
        print(f"Camera: {row['channel']}, Image Count: {row['image_count']}")


def main():
    """
    Main function to extract camera information from the database.
    """
    # Ensure the database file exists
    if not os.path.exists(DB_FILE):
        print(f"Error: Database file {DB_FILE} not found!")
        return
    
    # Get general camera information
    get_camera_info(DB_FILE)
    
    # Get image count by camera
    get_image_count_by_camera(DB_FILE)
    
    # Get information about some images
    get_image_info(DB_FILE)
    
    # Get calibration for a specific camera (if available)
    # First, try to find a camera token
    camera_token = get_camera_token_by_channel(DB_FILE, "CAM_F0")
    if camera_token:
        get_camera_calibration(DB_FILE, camera_token)
    else:
        print("\nNo CAM_F0 camera found in the database.")


if __name__ == "__main__":
    main() 