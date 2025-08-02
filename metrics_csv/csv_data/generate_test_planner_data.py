#!/usr/bin/env python3
"""
Generate Test Planner CSV Data

Creates realistic CSV planner data suitable for nuPlan evaluation system testing.
Based on the structure found in enhanced_trajectories_with_horizons.csv.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import pandas as pd
import numpy as np
import json
from pathlib import Path


def generate_realistic_planner_csv(
    num_scenarios: int = 5,
    timesteps_per_scenario: int = 100,
    horizon_length: int = 20,
    horizon_seconds: float = 2.0,
    output_file: str = "test_planner_data.csv"
) -> str:
    """
    Generate realistic planner CSV data with multiple scenarios.
    
    Args:
        num_scenarios: Number of different scenarios
        timesteps_per_scenario: Number of timesteps per scenario
        horizon_length: Number of points in each horizon
        horizon_seconds: Time duration of horizon in seconds
        output_file: Output CSV filename
        
    Returns:
        Path to generated CSV file
    """
    print(f"🎯 Generating realistic planner CSV data...")
    print(f"   Scenarios: {num_scenarios}")
    print(f"   Timesteps per scenario: {timesteps_per_scenario}")
    print(f"   Horizon length: {horizon_length}")
    print(f"   Total rows: {num_scenarios * timesteps_per_scenario}")
    
    data_rows = []
    
    # Scenario types from nuPlan
    scenario_types = [
        "stationary_at_traffic_light_with_lead",
        "traversing_traffic_light_intersection", 
        "on_stopline_crosswalk",
        "medium_magnitude_speed",
        "stationary_in_traffic"
    ]
    
    for scenario_idx in range(num_scenarios):
        scenario_type = scenario_types[scenario_idx % len(scenario_types)]
        scenario_id = f"scenario_{scenario_idx:04d}_{scenario_type[:8]}"
        nuplan_token = f"{scenario_idx:016x}"
        nuplan_log = f"2021.07.16.18.06.21_veh-{38+scenario_idx}_{scenario_idx:05d}_{scenario_idx+500:05d}"
        
        print(f"   Creating scenario {scenario_idx+1}/{num_scenarios}: {scenario_type}")
        
        # Base trajectory parameters for this scenario
        if scenario_type == "stationary_at_traffic_light_with_lead":
            # Mostly stationary with small movements
            base_velocity = 0.5
            trajectory_type = "stationary"
        elif scenario_type == "traversing_traffic_light_intersection":
            # Accelerating through intersection
            base_velocity = 8.0
            trajectory_type = "accelerating"
        elif scenario_type == "on_stopline_crosswalk":
            # Stopping then proceeding
            base_velocity = 5.0
            trajectory_type = "stop_and_go"
        elif scenario_type == "medium_magnitude_speed":
            # Steady highway driving
            base_velocity = 15.0
            trajectory_type = "highway"
        else:  # stationary_in_traffic
            # Stop and go traffic
            base_velocity = 3.0
            trajectory_type = "traffic"
        
        # Generate trajectory for this scenario
        dt = 0.1  # 10 Hz sampling
        start_time = 1625851234000000 + scenario_idx * 100000000  # microseconds
        
        # Base position (different for each scenario)
        base_x = 664590.0 + scenario_idx * 100.0
        base_y = 3997900.0 + scenario_idx * 50.0
        base_heading = scenario_idx * 0.5
        
        for step in range(timesteps_per_scenario):
            t = step * dt
            timestamp_us = start_time + int(step * dt * 1e6)
            
            # Generate realistic ego state based on trajectory type
            if trajectory_type == "stationary":
                # Small random movements around base position
                ego_x = base_x + np.random.normal(0, 0.1)
                ego_y = base_y + np.random.normal(0, 0.1)
                ego_heading = base_heading + np.random.normal(0, 0.01)
                ego_velocity_x = np.random.normal(0, 0.1)
                ego_velocity_y = np.random.normal(0, 0.05)
                
            elif trajectory_type == "accelerating":
                # Accelerating motion
                acceleration = 2.0 if t < 3.0 else 0.0
                velocity = min(base_velocity + acceleration * t, 12.0)
                ego_x = base_x + velocity * t + 0.5 * acceleration * t**2
                ego_y = base_y + 0.1 * t
                ego_heading = base_heading + 0.01 * t
                ego_velocity_x = velocity * np.cos(ego_heading)
                ego_velocity_y = velocity * np.sin(ego_heading)
                
            elif trajectory_type == "stop_and_go":
                # Stop then go pattern
                if t < 2.0:
                    # Deceleration to stop
                    velocity = max(base_velocity * (1 - t/2.0), 0)
                elif t < 4.0:
                    # Stationary
                    velocity = 0
                else:
                    # Acceleration
                    velocity = min(base_velocity * (t - 4.0) / 2.0, base_velocity)
                
                ego_x = base_x + np.cumsum([velocity * dt] * (step + 1))[-1]
                ego_y = base_y + 0.05 * t
                ego_heading = base_heading + 0.005 * t
                ego_velocity_x = velocity * np.cos(ego_heading)
                ego_velocity_y = velocity * np.sin(ego_heading)
                
            elif trajectory_type == "highway":
                # Steady highway driving
                ego_x = base_x + base_velocity * t
                ego_y = base_y + 0.5 * np.sin(0.1 * t)  # Slight lane changes
                ego_heading = base_heading + 0.1 * np.sin(0.1 * t)
                ego_velocity_x = base_velocity * np.cos(ego_heading)
                ego_velocity_y = base_velocity * np.sin(ego_heading) * 0.1
                
            else:  # traffic
                # Stop and go traffic pattern
                velocity_pattern = base_velocity * (0.5 + 0.5 * np.sin(0.5 * t))
                ego_x = base_x + np.sum([velocity_pattern * dt] * step) if step > 0 else base_x
                ego_y = base_y + 0.02 * t
                ego_heading = base_heading + 0.002 * t
                ego_velocity_x = velocity_pattern * np.cos(ego_heading)
                ego_velocity_y = velocity_pattern * np.sin(ego_heading) * 0.1
            
            # Generate accelerations
            ego_acceleration_x = np.random.normal(0, 0.5)
            ego_acceleration_y = np.random.normal(0, 0.2)
            
            # Generate steering angle
            tire_steering_angle = np.random.normal(0, 0.1)
            
            # Generate horizon trajectory
            horizon_dt = horizon_seconds / horizon_length
            horizon_x = []
            horizon_y = []
            horizon_heading = []
            horizon_velocity_x = []
            horizon_velocity_y = []
            horizon_acceleration_x = []
            horizon_acceleration_y = []
            
            for h in range(horizon_length):
                future_t = t + (h + 1) * horizon_dt
                
                # Project future state based on current motion
                if trajectory_type == "stationary":
                    future_x = ego_x + np.random.normal(0, 0.05)
                    future_y = ego_y + np.random.normal(0, 0.05)
                    future_heading = ego_heading + np.random.normal(0, 0.005)
                    future_vx = np.random.normal(0, 0.05)
                    future_vy = np.random.normal(0, 0.02)
                    
                elif trajectory_type == "accelerating":
                    future_velocity = min(base_velocity + 2.0 * future_t, 12.0)
                    future_x = ego_x + future_velocity * horizon_dt * (h + 1)
                    future_y = ego_y + 0.1 * horizon_dt * (h + 1)
                    future_heading = ego_heading + 0.01 * horizon_dt * (h + 1)
                    future_vx = future_velocity * np.cos(future_heading)
                    future_vy = future_velocity * np.sin(future_heading)
                    
                else:
                    # General future projection
                    future_x = ego_x + ego_velocity_x * horizon_dt * (h + 1)
                    future_y = ego_y + ego_velocity_y * horizon_dt * (h + 1)
                    future_heading = ego_heading + 0.01 * horizon_dt * (h + 1)
                    future_vx = ego_velocity_x * (1 + 0.1 * np.random.normal())
                    future_vy = ego_velocity_y * (1 + 0.1 * np.random.normal())
                
                future_ax = np.random.normal(0, 0.3)
                future_ay = np.random.normal(0, 0.1)
                
                horizon_x.append(future_x)
                horizon_y.append(future_y)
                horizon_heading.append(future_heading)
                horizon_velocity_x.append(future_vx)
                horizon_velocity_y.append(future_vy)
                horizon_acceleration_x.append(future_ax)
                horizon_acceleration_y.append(future_ay)
            
            # Create data row
            row = {
                'nuplan_token': nuplan_token,
                'nuplan_log': nuplan_log,
                'scenario_id': scenario_id,
                'planning_iteration': step,
                'data_iteration': step * 20,  # Simulate data iteration
                'timestamp_us': timestamp_us,
                'scenario_type': scenario_type,
                'ego_x': ego_x,
                'ego_y': ego_y,
                'ego_heading': ego_heading,
                'ego_velocity_x': ego_velocity_x,
                'ego_velocity_y': ego_velocity_y,
                'ego_acceleration_x': ego_acceleration_x,
                'ego_acceleration_y': ego_acceleration_y,
                'tire_steering_angle': tire_steering_angle,
                'horizon_seconds': horizon_seconds,
                'horizon_length': horizon_length,
                'horizon_x': json.dumps(horizon_x),
                'horizon_y': json.dumps(horizon_y),
                'horizon_heading': json.dumps(horizon_heading),
                'horizon_velocity_x': json.dumps(horizon_velocity_x),
                'horizon_velocity_y': json.dumps(horizon_velocity_y),
                'horizon_acceleration_x': json.dumps(horizon_acceleration_x),
                'horizon_acceleration_y': json.dumps(horizon_acceleration_y)
            }
            
            data_rows.append(row)
    
    # Create DataFrame and save
    df = pd.DataFrame(data_rows)
    output_path = Path(__file__).parent / output_file
    df.to_csv(output_path, index=False)
    
    print(f"✅ Generated CSV with {len(df)} rows and {len(df.columns)} columns")
    print(f"💾 Saved to: {output_path}")
    
    # Print summary
    print(f"\n📊 Data Summary:")
    print(f"   Unique scenarios: {df['scenario_id'].nunique()}")
    print(f"   Scenario types: {list(df['scenario_type'].unique())}")
    print(f"   Time range: {df['timestamp_us'].min()} to {df['timestamp_us'].max()}")
    print(f"   Position range X: {df['ego_x'].min():.1f} to {df['ego_x'].max():.1f}")
    print(f"   Position range Y: {df['ego_y'].min():.1f} to {df['ego_y'].max():.1f}")
    print(f"   Velocity range: {np.sqrt(df['ego_velocity_x']**2 + df['ego_velocity_y']**2).min():.1f} to {np.sqrt(df['ego_velocity_x']**2 + df['ego_velocity_y']**2).max():.1f} m/s")
    
    return str(output_path)


def main():
    """Main function to generate test data."""
    print("🚀 Test Planner Data Generator")
    print("=" * 50)
    
    # Generate comprehensive test dataset
    csv_file = generate_realistic_planner_csv(
        num_scenarios=5,
        timesteps_per_scenario=50,  # 5 seconds per scenario at 10Hz
        horizon_length=20,          # 2 second horizon
        horizon_seconds=2.0,
        output_file="test_planner_data.csv"
    )
    
    print(f"\n🎉 Test data generation complete!")
    print(f"📁 CSV file ready for evaluation: {csv_file}")
    
    # Display first few rows
    df = pd.read_csv(csv_file)
    print(f"\n📋 Sample data (first 3 rows):")
    print(df[['scenario_id', 'scenario_type', 'planning_iteration', 'ego_x', 'ego_y', 'ego_heading']].head(3))


if __name__ == "__main__":
    main()