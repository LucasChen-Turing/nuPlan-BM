# Extracting Data from nuPlan Database Files

This README provides information on how to extract data from nuPlan database (.db) files, which are SQLite databases containing driving log data.

## Database Structure

nuPlan database files are SQLite databases that contain structured information about driving logs, including:

- Ego vehicle poses and trajectories
- LiDAR point clouds (metadata, actual data stored separately)
- Camera images (metadata, actual images stored separately)
- Tracked objects (bounding boxes, classifications)
- Scenes and scenarios
- Traffic light status
- Map information

The schema includes tables such as:
- `log`: Information about the log file itself
- `lidar_pc`: LiDAR point cloud metadata
- `lidar_box`: Bounding boxes for tracked objects
- `ego_pose`: Ego vehicle poses
- `camera`: Camera calibration and metadata
- `image`: Image metadata
- `scene`: Scene information
- `track`: Tracked object information
- `category`: Object categories
- `traffic_light_status`: Traffic light states

## Approaches to Extract Data

There are three main approaches to extract data from nuPlan database files:

### 1. Direct SQL Queries

The simplest approach is to use direct SQL queries with the SQLite API. This works well for extracting metadata and doesn't require the full nuPlan devkit.

Example:
```python
import sqlite3

def execute_query(query, params, db_file):
    conn = sqlite3.connect(db_file)
    conn.row_factory = sqlite3.Row
    cursor = conn.cursor()
    
    try:
        cursor.execute(query, params)
        return cursor.fetchall()
    finally:
        cursor.close()
        conn.close()

# Example: Get ego vehicle trajectory
query = """
SELECT ep.timestamp, ep.x, ep.y, ep.z, ep.vx, ep.vy, ep.vz
FROM ego_pose AS ep
ORDER BY ep.timestamp
LIMIT 10;
"""

results = execute_query(query, (), "path/to/database.db")
for row in results:
    print(f"Position: ({row['x']}, {row['y']}, {row['z']}), "
          f"Velocity: ({row['vx']}, {row['vy']}, {row['vz']}), "
          f"Timestamp: {row['timestamp']}")
```

### 2. nuPlan DB Query Session API

The nuPlan devkit provides a query session API that simplifies SQL queries:

```python
from nuplan.database.nuplan_db.query_session import execute_one, execute_many

# Example: Count lidar point clouds
query = "SELECT COUNT(*) AS cnt FROM lidar_pc;"
result = execute_one(query, (), "path/to/database.db")
print(f"Number of lidar point clouds: {result['cnt']}")
```

### 3. nuPlan DB ORM API

For more complex interactions, the nuPlan devkit provides an Object-Relational Mapping (ORM) API:

```python
from nuplan.database.nuplan_db_orm.nuplandb import NuPlanDB

# Initialize database
db = NuPlanDB(data_root="path/to/data", db_path="database.db", verbose=False)

# Access tables through the ORM
print(f"Number of lidar_pc records: {len(db.lidar_pc)}")

# Get a sample lidar_pc and access related data
sample_lidar_pc = db.lidar_pc[0]
ego_pose = sample_lidar_pc.ego_pose
print(f"Ego pose: Position: ({ego_pose.x}, {ego_pose.y}, {ego_pose.z})")
```

### 4. nuPlan Scenario API

For higher-level access to scenario data, the nuPlan devkit provides a scenario API:

```python
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario import NuPlanScenario
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters

# Create a scenario
scenario = NuPlanScenario(
    data_root="path/to/data",
    log_file_load_path="path/to/database.db",
    initial_lidar_token="token_hex_string",
    initial_lidar_timestamp=timestamp_integer,
    scenario_type="scenario_type",
    map_root="path/to/maps",
    map_version="map_version",
    map_name="map_name",
    ego_vehicle_parameters=get_pacifica_parameters(),
    sensor_root="path/to/sensor_data"
)

# Access scenario data
ego_state = scenario.get_ego_state_at_iteration(0)
tracked_objects = scenario.get_tracked_objects_at_iteration(0)
```

## Accessing Sensor Data

The database files contain metadata about sensor data, but the actual sensor data (LiDAR point clouds, camera images) are stored separately in the `sensor_blobs` directory. The mini dataset typically doesn't include this data.

### LiDAR Point Clouds

To access LiDAR point clouds:
1. Get the filename from the `lidar_pc` table
2. Construct the path to the point cloud file in the `sensor_blobs` directory
3. Load the point cloud file (typically in PCD format)

### Camera Images

To access camera images:
1. Get the filename from the `image` table
2. Construct the path to the image file in the `sensor_blobs` directory
3. Load the image file (typically in JPG format)

## Example Scripts

This repository includes several example scripts for extracting data from nuPlan database files:

- `extract_nuplan_data.py`: Demonstrates basic data extraction using direct SQL queries
- `extract_pointcloud.py`: Shows how to extract LiDAR point cloud metadata
- `extract_camera_info.py`: Extracts camera information and image metadata

## Requirements

- For basic SQL queries: Python with SQLite support
- For nuPlan API: Full nuPlan devkit installation

## References

- [nuPlan devkit repository](https://github.com/motional/nuplan-devkit)
- [nuPlan schema documentation](https://github.com/motional/nuplan-devkit/blob/master/docs/nuplan_schema.md)
- [nuPlan tutorials](https://github.com/motional/nuplan-devkit/tree/master/tutorials) 