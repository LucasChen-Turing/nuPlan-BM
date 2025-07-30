# CSV Trajectory Data Schema

## Overview
This document defines the CSV data format for offline trajectory evaluation in nuPlan.

## Required CSV Columns

### Temporal Information
- `timestamp_us` (int64): Timestamp in microseconds
- `iteration` (int): Step number in trajectory (0-based)

### Ego Vehicle Pose (Center Position)
- `ego_x` (float64): X coordinate in meters (center of vehicle)
- `ego_y` (float64): Y coordinate in meters (center of vehicle) 
- `ego_heading` (float64): Heading angle in radians

### Ego Vehicle Dynamics
- `ego_velocity_x` (float64): Velocity in X direction (m/s)
- `ego_velocity_y` (float64): Velocity in Y direction (m/s)
- `ego_acceleration_x` (float64): Acceleration in X direction (m/s²)
- `ego_acceleration_y` (float64): Acceleration in Y direction (m/s²)
- `ego_angular_velocity` (float64): Angular velocity (rad/s)
- `ego_angular_acceleration` (float64): Angular acceleration (rad/s²)

### Vehicle Control
- `tire_steering_angle` (float64): Steering angle in radians

### Scenario Metadata  
- `scenario_id` (str): Unique identifier for this trajectory
- `scenario_type` (str): Type of scenario (e.g., "straight", "lane_change", "turn_left")

## Data Requirements

### Frequency and Duration
- **Sampling Rate**: 4Hz (0.25 second intervals)
- **Default Horizon**: 10 seconds (40 data points per trajectory)
- **Minimum Trajectory Length**: 2 seconds (8 data points)

### Coordinate System
- Uses nuPlan's standard coordinate system
- Origin at map reference point
- X-axis points East, Y-axis points North
- Heading: 0 = East, π/2 = North (counter-clockwise positive)

### Units
- Distance: meters
- Time: microseconds for timestamps, seconds for durations
- Angles: radians
- Velocity: m/s
- Acceleration: m/s²

## Example CSV Structure

```csv
timestamp_us,iteration,ego_x,ego_y,ego_heading,ego_velocity_x,ego_velocity_y,ego_acceleration_x,ego_acceleration_y,ego_angular_velocity,ego_angular_acceleration,tire_steering_angle,scenario_id,scenario_type
1621720800000000,0,100.0,50.0,0.0,5.0,0.0,0.0,0.0,0.0,0.0,0.0,traj_001,straight
1621720800250000,1,101.25,50.0,0.0,5.0,0.0,0.0,0.0,0.0,0.0,0.0,traj_001,straight
1621720800500000,2,102.5,50.0,0.0,5.0,0.0,0.0,0.0,0.0,0.0,0.0,traj_001,straight
```

## Validation Rules

1. **Temporal Consistency**: Timestamps must be monotonically increasing
2. **Physical Constraints**: 
   - Maximum velocity: 30 m/s (highway speeds)
   - Maximum acceleration: ±5 m/s²
   - Maximum steering angle: ±0.6 radians (~35 degrees)
3. **Trajectory Continuity**: Position and velocity should be smooth between timesteps
4. **Minimum Length**: Each trajectory must have at least 8 data points (2 seconds)

## Notes for Implementation

- CSV planner will read one trajectory at a time based on scenario_id
- Multiple trajectories can be stored in same CSV file
- Missing values should be handled gracefully (use previous value or interpolate)
- The planner will convert this data to nuPlan's EgoState format internally