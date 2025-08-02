#!/usr/bin/env python3
"""
Generate enhanced CSV data with receding horizons for longer scenarios.
This creates proper Challenge 1 data with sufficient timesteps for 8s horizon evaluation.
"""

import json
import pandas as pd
import numpy as np
from pathlib import Path

def generate_enhanced_csv_for_longer_scenario():
    """Generate enhanced CSV data for scenario_000 with 32 timesteps."""
    
    # Read the original scenario data
    input_file = "/home/chen/nuplan-devkit/offline_evaluation/data/real_scenario_tokens.csv"
    df = pd.read_csv(input_file)
    
    # Filter for scenario_000 (32 timesteps, low_magnitude_speed)
    scenario_data = df[df['scenario_id'] == 'scenario_000'].copy()
    
    print(f"Processing scenario_000 with {len(scenario_data)} timesteps")
    print(f"Token: {scenario_data.iloc[0]['nuplan_token']}")
    print(f"Log: {scenario_data.iloc[0]['nuplan_log']}")
    
    # Generate enhanced data with horizons
    enhanced_data = []
    
    # Configuration
    planning_frequency = 1.0  # Hz
    data_frequency = 1.0  # Hz (our actual data frequency)
    horizon_seconds = 6.0
    horizon_steps = int(horizon_seconds * data_frequency)  # 6 steps at 1Hz
    
    # We'll create 6-step horizons and then interpolate to 20Hz for nuPlan compatibility
    
    for planning_step in range(len(scenario_data)):
        current_row = scenario_data.iloc[planning_step]
        
        # Calculate how many future timesteps we can include
        remaining_timesteps = len(scenario_data) - planning_step - 1
        horizon_timesteps = min(horizon_steps, remaining_timesteps)
        
        if horizon_timesteps < 3:  # At least 3 seconds of horizon
            print(f"Skipping planning step {planning_step} - insufficient future data ({horizon_timesteps} timesteps)")
            continue
            
        # Get future horizon data
        end_idx = min(planning_step + 1 + horizon_timesteps, len(scenario_data))
        future_data = scenario_data.iloc[planning_step + 1:end_idx]
        
        # Extract horizon arrays (1Hz data)
        horizon_x_1hz = future_data['ego_x'].tolist()
        horizon_y_1hz = future_data['ego_y'].tolist()
        horizon_heading_1hz = future_data['ego_heading'].tolist()
        horizon_velocity_x_1hz = future_data['ego_velocity_x'].tolist()
        horizon_velocity_y_1hz = future_data['ego_velocity_y'].tolist()
        horizon_acceleration_x_1hz = future_data['ego_acceleration_x'].tolist()
        horizon_acceleration_y_1hz = future_data['ego_acceleration_y'].tolist()
        
        # Interpolate to exactly 120 points for 6 seconds at 20Hz
        def interpolate_to_20hz(values_1hz):
            """Interpolate 1Hz data to exactly 120 points (6s at 20Hz)."""
            if len(values_1hz) == 0:
                return []
            
            # Generate exactly 120 evenly spaced time points over 6 seconds
            target_points = 120
            time_points = np.linspace(0, len(values_1hz) - 1, target_points)
            
            # Interpolate values at these time points
            interpolated = []
            values_array = np.array(values_1hz)
            
            for t in time_points:
                if t <= 0:
                    interpolated.append(values_array[0])
                elif t >= len(values_1hz) - 1:
                    # For points beyond our data, extrapolate using the last two points
                    if len(values_1hz) >= 2:
                        # Linear extrapolation
                        last_val = values_array[-1]
                        second_last_val = values_array[-2]
                        extrapolation_factor = t - (len(values_1hz) - 1)
                        slope = last_val - second_last_val
                        extrapolated_val = last_val + slope * extrapolation_factor
                        interpolated.append(extrapolated_val)
                    else:
                        interpolated.append(values_array[-1])
                else:
                    # Linear interpolation between adjacent points
                    lower_idx = int(t)
                    upper_idx = min(lower_idx + 1, len(values_1hz) - 1)
                    alpha = t - lower_idx
                    interp_val = values_array[lower_idx] * (1 - alpha) + values_array[upper_idx] * alpha
                    interpolated.append(interp_val)
            
            return interpolated
        
        # Interpolate all horizon arrays to 20Hz
        horizon_x = interpolate_to_20hz(horizon_x_1hz)
        horizon_y = interpolate_to_20hz(horizon_y_1hz)
        horizon_heading = interpolate_to_20hz(horizon_heading_1hz)
        horizon_velocity_x = interpolate_to_20hz(horizon_velocity_x_1hz)
        horizon_velocity_y = interpolate_to_20hz(horizon_velocity_y_1hz)
        horizon_acceleration_x = interpolate_to_20hz(horizon_acceleration_x_1hz)
        horizon_acceleration_y = interpolate_to_20hz(horizon_acceleration_y_1hz)
        
        # Ensure we have at least some horizon data
        if len(horizon_x) == 0:
            continue
        
        # Create enhanced row
        enhanced_row = {
            'nuplan_token': current_row['nuplan_token'],
            'nuplan_log': current_row['nuplan_log'],
            'scenario_id': f"scenario_{current_row['nuplan_token'][:16]}",
            'planning_iteration': planning_step,
            'data_iteration': current_row['iteration'],
            'timestamp_us': current_row['timestamp_us'],
            'scenario_type': current_row['scenario_type'],
            'ego_x': current_row['ego_x'],
            'ego_y': current_row['ego_y'],
            'ego_heading': current_row['ego_heading'],
            'ego_velocity_x': current_row['ego_velocity_x'],
            'ego_velocity_y': current_row['ego_velocity_y'],
            'ego_acceleration_x': current_row['ego_acceleration_x'],
            'ego_acceleration_y': current_row['ego_acceleration_y'],
            'tire_steering_angle': current_row.get('tire_steering_angle', 0.0),
            'horizon_seconds': horizon_seconds,
            'horizon_length': len(horizon_x),
            'horizon_x': json.dumps(horizon_x),
            'horizon_y': json.dumps(horizon_y),
            'horizon_heading': json.dumps(horizon_heading),
            'horizon_velocity_x': json.dumps(horizon_velocity_x),
            'horizon_velocity_y': json.dumps(horizon_velocity_y),
            'horizon_acceleration_x': json.dumps(horizon_acceleration_x),
            'horizon_acceleration_y': json.dumps(horizon_acceleration_y)
        }
        
        enhanced_data.append(enhanced_row)
        print(f"Planning step {planning_step}: {len(horizon_x)} horizon points")
    
    # Create DataFrame and save
    enhanced_df = pd.DataFrame(enhanced_data)
    output_file = "/home/chen/nuplan-devkit/offline_evaluation/data/longer_scenario_horizons.csv"
    enhanced_df.to_csv(output_file, index=False)
    
    print(f"\nGenerated enhanced CSV with {len(enhanced_df)} planning timesteps")
    print(f"Saved to: {output_file}")
    if len(enhanced_df) > 0:
        print(f"Scenario token: {enhanced_df.iloc[0]['nuplan_token']}")
        print(f"Scenario log: {enhanced_df.iloc[0]['nuplan_log']}")
    else:
        print("No valid planning timesteps generated!")
    
    return output_file

if __name__ == "__main__":
    generate_enhanced_csv_for_longer_scenario()