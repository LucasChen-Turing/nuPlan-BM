#!/usr/bin/env python3
"""
Generate Sample Trajectory Data

Creates sample CSV trajectory data for testing the offline evaluation system.
This script generates various trajectory patterns compatible with nuPlan.
"""

import numpy as np
import pandas as pd
from pathlib import Path
import argparse


def generate_straight_trajectory(scenario_id: str, duration: float = 8.0, velocity: float = 10.0) -> pd.DataFrame:
    """Generate a straight-line trajectory."""
    timesteps = int(duration * 4)  # 4Hz sampling
    
    data = []
    for i in range(timesteps):
        t = i * 0.25  # 0.25s intervals
        timestamp_us = int(t * 1e6)
        
        data.append({
            'scenario_id': scenario_id,
            'iteration': i,
            'timestamp_us': timestamp_us,
            'ego_x': velocity * t,  # Moving forward at constant velocity
            'ego_y': 0.0,
            'ego_heading': 0.0,
            'ego_velocity_x': velocity,
            'ego_velocity_y': 0.0,
            'ego_acceleration_x': 0.0,
            'ego_acceleration_y': 0.0,
            'ego_angular_velocity': 0.0,
            'ego_angular_acceleration': 0.0,
            'tire_steering_angle': 0.0,
            'scenario_type': 'straight'
        })
    
    return pd.DataFrame(data)


def generate_turn_trajectory(scenario_id: str, duration: float = 8.0, radius: float = 20.0, angular_velocity: float = 0.2) -> pd.DataFrame:
    """Generate a turning trajectory."""
    timesteps = int(duration * 4)  # 4Hz sampling
    linear_velocity = radius * angular_velocity
    
    data = []
    for i in range(timesteps):
        t = i * 0.25  # 0.25s intervals
        timestamp_us = int(t * 1e6)
        
        # Circular motion
        angle = angular_velocity * t
        x = radius * np.sin(angle)
        y = radius * (1 - np.cos(angle))
        heading = angle
        
        # Velocity in global frame
        vx = linear_velocity * np.cos(angle)
        vy = linear_velocity * np.sin(angle)
        
        data.append({
            'scenario_id': scenario_id,
            'iteration': i,
            'timestamp_us': timestamp_us,
            'ego_x': x,
            'ego_y': y,
            'ego_heading': heading,
            'ego_velocity_x': vx,
            'ego_velocity_y': vy,
            'ego_acceleration_x': -angular_velocity * vy,  # Centripetal acceleration
            'ego_acceleration_y': angular_velocity * vx,
            'ego_angular_velocity': angular_velocity,
            'ego_angular_acceleration': 0.0,
            'tire_steering_angle': 0.1,  # Small steering angle
            'scenario_type': 'turn'
        })
    
    return pd.DataFrame(data)


def generate_lane_change_trajectory(scenario_id: str, duration: float = 8.0, lane_width: float = 3.5) -> pd.DataFrame:
    """Generate a lane change trajectory."""
    timesteps = int(duration * 4)  # 4Hz sampling
    velocity = 15.0  # m/s
    
    # Lane change happens in middle 4 seconds
    change_start = 2.0
    change_end = 6.0
    
    data = []
    for i in range(timesteps):
        t = i * 0.25  # 0.25s intervals
        timestamp_us = int(t * 1e6)
        
        # Forward motion
        x = velocity * t
        
        # Lateral motion (lane change)
        if change_start <= t <= change_end:
            # Smooth lane change using sinusoidal profile
            progress = (t - change_start) / (change_end - change_start)
            y = lane_width * 0.5 * (1 - np.cos(np.pi * progress))
            vy = (lane_width * np.pi * np.sin(np.pi * progress)) / (2 * (change_end - change_start))
            heading = np.arctan2(vy, velocity)
        else:
            y = lane_width if t > change_end else 0.0
            vy = 0.0
            heading = 0.0
        
        data.append({
            'scenario_id': scenario_id,
            'iteration': i,
            'timestamp_us': timestamp_us,
            'ego_x': x,
            'ego_y': y,
            'ego_heading': heading,
            'ego_velocity_x': velocity,
            'ego_velocity_y': vy,
            'ego_acceleration_x': 0.0,
            'ego_acceleration_y': 0.0,
            'ego_angular_velocity': 0.0,
            'ego_angular_acceleration': 0.0,
            'tire_steering_angle': 0.05 if change_start <= t <= change_end else 0.0,
            'scenario_type': 'lane_change'
        })
    
    return pd.DataFrame(data)


def generate_acceleration_trajectory(scenario_id: str, duration: float = 8.0, initial_velocity: float = 5.0, acceleration: float = 2.0) -> pd.DataFrame:
    """Generate an acceleration trajectory."""
    timesteps = int(duration * 4)  # 4Hz sampling
    
    data = []
    for i in range(timesteps):
        t = i * 0.25  # 0.25s intervals
        timestamp_us = int(t * 1e6)
        
        # Kinematic equations
        velocity = initial_velocity + acceleration * t
        x = initial_velocity * t + 0.5 * acceleration * t**2
        
        data.append({
            'scenario_id': scenario_id,
            'iteration': i,
            'timestamp_us': timestamp_us,
            'ego_x': x,
            'ego_y': 0.0,
            'ego_heading': 0.0,
            'ego_velocity_x': velocity,
            'ego_velocity_y': 0.0,
            'ego_acceleration_x': acceleration,
            'ego_acceleration_y': 0.0,
            'ego_angular_velocity': 0.0,
            'ego_angular_acceleration': 0.0,
            'tire_steering_angle': 0.0,
            'scenario_type': 'acceleration'
        })
    
    return pd.DataFrame(data)


def generate_sample_dataset(output_file: str, num_scenarios_per_type: int = 2) -> None:
    """Generate a complete sample dataset with various trajectory types."""
    all_trajectories = []
    scenario_counter = 1
    
    # Generate different trajectory types
    generators = [
        ("straight", generate_straight_trajectory),
        ("turn", generate_turn_trajectory),
        ("lane_change", generate_lane_change_trajectory),
        ("acceleration", generate_acceleration_trajectory)
    ]
    
    for traj_type, generator_func in generators:
        for i in range(num_scenarios_per_type):
            scenario_id = f"{traj_type}_{scenario_counter:03d}"
            
            if traj_type == "straight":
                df = generator_func(scenario_id, velocity=8 + i * 2)
            elif traj_type == "turn":
                df = generator_func(scenario_id, radius=15 + i * 5, angular_velocity=0.15 + i * 0.05)
            elif traj_type == "lane_change":
                df = generator_func(scenario_id, lane_width=3.5)
            elif traj_type == "acceleration":
                df = generator_func(scenario_id, initial_velocity=3 + i * 2, acceleration=1.5 + i * 0.5)
            
            all_trajectories.append(df)
            scenario_counter += 1
            
            print(f"Generated {traj_type} trajectory: {scenario_id}")
    
    # Combine all trajectories
    combined_df = pd.concat(all_trajectories, ignore_index=True)
    
    # Save to CSV
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    combined_df.to_csv(output_path, index=False)
    
    print(f"\nGenerated sample dataset:")
    print(f"  Total trajectories: {len(all_trajectories)}")
    print(f"  Total data points: {len(combined_df)}")
    print(f"  Saved to: {output_path}")
    print(f"  File size: {output_path.stat().st_size / 1024:.1f} KB")
    
    # Print summary statistics
    print(f"\nScenario summary:")
    scenario_counts = combined_df.groupby('scenario_type').size()
    for scenario_type, count in scenario_counts.items():
        unique_scenarios = combined_df[combined_df['scenario_type'] == scenario_type]['scenario_id'].nunique()
        print(f"  {scenario_type}: {unique_scenarios} scenarios, {count} data points")


def main():
    """Main function to generate sample trajectories."""
    parser = argparse.ArgumentParser(description="Generate sample trajectory data for offline evaluation")
    parser.add_argument(
        "--output", "-o",
        default="/home/chen/nuplan-devkit/offline_evaluation/data/sample_trajectories.csv",
        help="Output CSV file path"
    )
    parser.add_argument(
        "--scenarios-per-type", "-n",
        type=int, default=2,
        help="Number of scenarios to generate per trajectory type"
    )
    
    args = parser.parse_args()
    
    generate_sample_dataset(args.output, args.scenarios_per_type)
    
    print(f"\nSample trajectory generation completed!")
    print(f"You can now run offline evaluation with:")
    print(f"  python offline_evaluation/scripts/run_offline_evaluation.py --csv-file {args.output} --simple-mode")


if __name__ == "__main__":
    main()