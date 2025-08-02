#!/usr/bin/env python3
"""
Generate Enhanced CSV Data with Receding Horizons for Challenge 1

This script creates CSV data where each timestep contains not just the current state,
but also the future horizon predictions (6+ seconds) needed for Challenge 1 metrics.

Challenge 1 Requirements:
- 1Hz planning frequency (planner outputs every 1 second)
- 6+ second planning horizon (planner must predict 6+ seconds ahead)
- Metrics compare these predictions against actual expert trajectory

CSV Structure Enhancement:
- Each row represents a planning timestep (1Hz frequency)
- Each row contains current state + 6s horizon prediction
- Horizon stored as JSON arrays for positions, headings, velocities
"""

import os
import sys
import json
import logging
import pandas as pd
import numpy as np
from pathlib import Path

# Add nuplan to path
sys.path.append('/home/chen/nuplan-devkit')

from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
from nuplan.planning.utils.multithreading.worker_parallel import SingleMachineParallelExecutor

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Generate enhanced CSV data with receding horizons for Challenge 1."""
    
    # Database path - use working databases
    db_path = "/home/chen/nuplan-devkit/data/cache/mini_working"
    map_root = "/home/chen/nuplan-devkit/maps"
    
    # Set environment
    os.environ['NUPLAN_DATA_ROOT'] = db_path
    os.environ['NUPLAN_MAPS_ROOT'] = map_root
    
    print("🚀 Generating Enhanced CSV Data with Receding Horizons for Challenge 1")
    print(f"📊 Challenge 1 Requirements:")
    print(f"   - Planning Frequency: 1Hz (every 1 second)")
    print(f"   - Planning Horizon: 6+ seconds ahead")
    print(f"   - Metrics: ADE/FDE/Miss Rate/AHE/FHE within bound")
    print()
    
    # Create scenario builder
    scenario_builder = NuPlanScenarioBuilder(
        data_root=db_path,
        map_root=map_root,
        sensor_root=db_path,
        db_files=None,
        map_version="nuplan-maps-v1.0"
    )
    
    # Find stationary scenarios (same as working simulation)
    scenario_filter = ScenarioFilter(
        scenario_types=['stationary'],
        scenario_tokens=None,
        log_names=None,
        map_names=None,
        num_scenarios_per_type=None,
        limit_total_scenarios=1,
        timestamp_threshold_s=None,
        ego_displacement_minimum_m=None,
        expand_scenarios=False,
        remove_invalid_goals=False,
        shuffle=False
    )
    
    scenarios = scenario_builder.get_scenarios(scenario_filter, SingleMachineParallelExecutor())
    
    if not scenarios:
        print("❌ No stationary scenarios found!")
        return
    
    scenario = scenarios[0]
    print(f"✅ Found working stationary scenario: {scenario.token}")
    print(f"   - Number of iterations: {scenario.get_number_of_iterations()}")
    
    # Extract expert trajectory
    expert_trajectory = list(scenario.get_expert_ego_trajectory())
    print(f"   - Expert trajectory length: {len(expert_trajectory)}")
    
    # Challenge 1 parameters
    planning_frequency_hz = 1.0  # 1Hz planning frequency
    data_frequency_hz = 20.0    # nuPlan data is at 20Hz
    horizon_seconds = 6.0       # Minimum 6 second horizon
    
    planning_interval_steps = int(data_frequency_hz / planning_frequency_hz)  # 20 steps = 1 second
    horizon_steps = int(horizon_seconds * data_frequency_hz)  # 120 steps = 6 seconds
    
    print(f"📈 Data Generation Parameters:")
    print(f"   - Planning interval: {planning_interval_steps} steps (1 second at 20Hz)")
    print(f"   - Horizon length: {horizon_steps} steps (6 seconds at 20Hz)")
    print()
    
    # Generate enhanced CSV data
    enhanced_data = []
    
    # Process at 1Hz intervals (every 20 steps in 20Hz data)
    for planning_step in range(0, len(expert_trajectory), planning_interval_steps):
        if planning_step + horizon_steps >= len(expert_trajectory):
            # Not enough data for full horizon, stop here
            break
        
        # Current state (at planning timestep)
        current_state = expert_trajectory[planning_step]
        
        # Future horizon (next 6 seconds = 120 steps)
        horizon_states = expert_trajectory[planning_step + 1:planning_step + 1 + horizon_steps]
        
        if len(horizon_states) < horizon_steps:
            # Not enough data for full horizon
            break
        
        # Extract horizon data
        horizon_x = [state.rear_axle.x for state in horizon_states]
        horizon_y = [state.rear_axle.y for state in horizon_states]
        horizon_heading = [state.rear_axle.heading for state in horizon_states]
        horizon_vx = [state.dynamic_car_state.rear_axle_velocity_2d.x for state in horizon_states]
        horizon_vy = [state.dynamic_car_state.rear_axle_velocity_2d.y for state in horizon_states]
        horizon_ax = [state.dynamic_car_state.rear_axle_acceleration_2d.x for state in horizon_states]
        horizon_ay = [state.dynamic_car_state.rear_axle_acceleration_2d.y for state in horizon_states]
        
        # Create enhanced CSV row
        enhanced_row = {
            # Standard fields
            'nuplan_token': scenario.token,
            'nuplan_log': scenario.log_name if hasattr(scenario, 'log_name') else 'expert_log',
            'scenario_id': f'scenario_{scenario.token}',
            'planning_iteration': planning_step // planning_interval_steps,  # Planning timestep index
            'data_iteration': planning_step,  # Original data iteration
            'timestamp_us': current_state.time_point.time_us,
            'scenario_type': 'stationary',
            
            # Current state
            'ego_x': current_state.rear_axle.x,
            'ego_y': current_state.rear_axle.y,
            'ego_heading': current_state.rear_axle.heading,
            'ego_velocity_x': current_state.dynamic_car_state.rear_axle_velocity_2d.x,
            'ego_velocity_y': current_state.dynamic_car_state.rear_axle_velocity_2d.y,
            'ego_acceleration_x': current_state.dynamic_car_state.rear_axle_acceleration_2d.x,
            'ego_acceleration_y': current_state.dynamic_car_state.rear_axle_acceleration_2d.y,
            'tire_steering_angle': getattr(current_state.tire_steering_angle, 'value', 0.0) if hasattr(current_state, 'tire_steering_angle') else 0.0,
            
            # Horizon predictions (JSON arrays)
            'horizon_seconds': horizon_seconds,
            'horizon_length': len(horizon_states),
            'horizon_x': json.dumps(horizon_x),
            'horizon_y': json.dumps(horizon_y),
            'horizon_heading': json.dumps(horizon_heading),
            'horizon_velocity_x': json.dumps(horizon_vx),
            'horizon_velocity_y': json.dumps(horizon_vy),
            'horizon_acceleration_x': json.dumps(horizon_ax),
            'horizon_acceleration_y': json.dumps(horizon_ay),
        }
        
        enhanced_data.append(enhanced_row)
    
    print(f"✅ Generated {len(enhanced_data)} planning timesteps with receding horizons")
    print(f"   - Each timestep: current state + {horizon_seconds}s horizon ({horizon_steps} future points)")
    print(f"   - Total horizon predictions: {len(enhanced_data) * horizon_steps}")
    
    # Save enhanced CSV
    df = pd.DataFrame(enhanced_data)
    output_path = "/home/chen/nuplan-devkit/offline_evaluation/data/enhanced_trajectories_with_horizons.csv"
    df.to_csv(output_path, index=False)
    
    print(f"💾 Saved enhanced CSV to: {output_path}")
    print(f"📊 CSV Structure:")
    print(f"   - Rows: {len(df)} (planning timesteps at 1Hz)")
    print(f"   - Columns: {len(df.columns)}")
    print(f"   - Key columns: current state + horizon_x/y/heading/velocity arrays")
    print()
    
    # Validate the data
    print("🔍 Data Validation:")
    sample_row = enhanced_data[0]
    horizon_x_sample = json.loads(sample_row['horizon_x'])
    print(f"   - First planning timestep has {len(horizon_x_sample)} horizon points")
    print(f"   - Horizon duration: {len(horizon_x_sample) / data_frequency_hz:.1f} seconds")
    print(f"   - Meets Challenge 1 requirement: {len(horizon_x_sample) / data_frequency_hz >= 6.0}")
    
    print()
    print("🎯 Ready for Challenge 1 Evaluation!")
    print("   - Enhanced CSV contains proper receding horizons")
    print("   - Compatible with Challenge 1 'within bound' metrics")
    print("   - ADE/FDE/Miss Rate/AHE/FHE metrics can now be computed")


if __name__ == "__main__":
    main()