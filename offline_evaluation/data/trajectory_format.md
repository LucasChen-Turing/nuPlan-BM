# CSV Trajectory Data Format

## Overview
This document specifies the CSV format for trajectory data used in offline evaluation.

## Required Columns

### Identification
- `scenario_id` (str): Unique identifier for each trajectory/scenario
- `iteration` (int): Timestep index (0-based, 4Hz sampling)
- `timestamp_us` (int64): Timestamp in microseconds

### Ego Vehicle Pose
- `ego_x` (float64): X coordinate in meters (vehicle center)
- `ego_y` (float64): Y coordinate in meters (vehicle center)
- `ego_heading` (float64): Heading angle in radians

### Ego Vehicle Dynamics
- `ego_velocity_x` (float64): Velocity in X direction (m/s)
- `ego_velocity_y` (float64): Velocity in Y direction (m/s)

### Optional Columns
- `ego_acceleration_x` (float64): Acceleration in X direction (m/s²)
- `ego_acceleration_y` (float64): Acceleration in Y direction (m/s²)
- `ego_angular_velocity` (float64): Angular velocity (rad/s)
- `ego_angular_acceleration` (float64): Angular acceleration (rad/s²)
- `tire_steering_angle` (float64): Steering angle in radians
- `scenario_type` (str): Type of scenario (optional, for analysis)

## Data Requirements

### Sampling and Duration
- **Frequency**: 4Hz (0.25 second intervals)
- **Typical Duration**: 8-10 seconds per trajectory (32-40 data points)
- **Minimum Length**: 2 seconds (8 data points)

### Coordinate System
- Uses nuPlan's standard coordinate system
- Origin at map reference point
- X-axis: East, Y-axis: North
- Heading: 0 = East, π/2 = North (counter-clockwise positive)

### Units
- Distance: meters
- Time: microseconds for timestamps
- Angles: radians
- Velocity: m/s
- Acceleration: m/s²

## Example CSV Structure

```csv
scenario_id,iteration,timestamp_us,ego_x,ego_y,ego_heading,ego_velocity_x,ego_velocity_y,ego_acceleration_x,ego_acceleration_y,tire_steering_angle,scenario_type
traj_001,0,1621720800000000,100.0,50.0,0.0,10.0,0.0,0.0,0.0,0.0,straight
traj_001,1,1621720800250000,102.5,50.0,0.0,10.0,0.0,0.0,0.0,0.0,straight
traj_001,2,1621720800500000,105.0,50.0,0.0,10.0,0.0,0.0,0.0,0.0,straight
traj_002,0,1621720800000000,200.0,100.0,0.0,8.0,0.0,2.0,0.0,0.0,acceleration
traj_002,1,1621720800250000,202.0,100.0,0.0,8.5,0.0,2.0,0.0,0.0,acceleration
```

## Validation Rules

1. **Temporal Consistency**: 
   - Timestamps must be monotonically increasing within each scenario
   - Iteration numbers must be sequential (0, 1, 2, ...)

2. **Physical Constraints**:
   - Maximum velocity: 30 m/s (highway speeds)
   - Maximum acceleration: ±5 m/s²
   - Maximum steering angle: ±0.6 radians (~35 degrees)

3. **Data Continuity**:
   - Position and velocity should be physically consistent between timesteps
   - No large discontinuous jumps in trajectory

4. **Completeness**:
   - Each scenario must have at least 8 data points
   - All required columns must be present and non-null

## Usage with nuPlan

The CSV data is converted to nuPlan's standard `EgoState` objects during simulation:
- Position → `CarFootprint` with vehicle center position
- Velocities → `DynamicCarState` with rear axle velocities
- Time → `TimePoint` objects for temporal queries
- All data integrated into `InterpolatedTrajectory` for smooth simulation

## Conversion from nuPlan Database

To extract trajectory data from existing nuPlan scenarios:

```python
# Example code to extract ego trajectory from nuPlan scenario
ego_states = scenario.get_ego_future_trajectory(0, 10.0, 40)
csv_data = []
for i, state in enumerate(ego_states):
    csv_data.append({
        'scenario_id': scenario.scenario_name,
        'iteration': i,
        'timestamp_us': state.time_point.time_us,
        'ego_x': state.car_footprint.center.x,
        'ego_y': state.car_footprint.center.y,
        'ego_heading': state.car_footprint.center.heading,
        'ego_velocity_x': state.dynamic_car_state.center_velocity_2d.x,
        'ego_velocity_y': state.dynamic_car_state.center_velocity_2d.y,
        # ... additional fields
    })
```