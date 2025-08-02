#!/usr/bin/env python3
"""
Debug script to analyze trajectory length mismatch in CSVLogFuturePlanner
"""

import os
import sys
import logging
from pathlib import Path

# Add nuplan to path
sys.path.append('/home/chen/nuplan-devkit')

from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_utils import ScenarioMapping
from nuplan.planning.utils.multithreading.worker_parallel import SingleMachineParallelExecutor
from offline_evaluation.planners.csv_log_future_planner import CSVLogFuturePlanner
from nuplan.planning.simulation.planner.log_future_planner import LogFuturePlanner
from nuplan.planning.simulation.observation.idm_agents import IDMAgents
from nuplan.planning.simulation.simulation_time_controller.step_simulation_time_controller import StepSimulationTimeController
from nuplan.planning.simulation.planner.abstract_planner import PlannerInitialization, PlannerInput

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def main():
    """Debug trajectory lengths for CSV vs Log planner"""
    
    # Database path - use working databases
    db_path = "/home/chen/nuplan-devkit/data/cache/mini_working"
    map_root = "/home/chen/nuplan-devkit/maps"
    
    # Set environment
    os.environ['NUPLAN_DATA_ROOT'] = db_path
    os.environ['NUPLAN_MAPS_ROOT'] = map_root
    
    print(f"Finding working stationary scenarios...")
    
    # Create scenario builder
    scenario_builder = NuPlanScenarioBuilder(
        data_root=db_path,
        map_root=map_root,
        sensor_root=db_path,  # Required parameter
        db_files=None,
        map_version="nuplan-maps-v1.0"
    )
    
    # Find stationary scenarios to match our working simulation
    from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
    
    # Get all scenarios and filter for stationary
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
    
    # No need for scenario mapping - we have the scenario directly
    
    try:
        print(f"✓ Successfully loaded scenario: {scenario.token}")
        print(f"  - Database interval: {scenario.database_interval}")
        print(f"  - Number of iterations: {scenario.get_number_of_iterations()}")
        
        # Get expert trajectory info
        expert_trajectory = list(scenario.get_expert_ego_trajectory())
        print(f"  - Expert trajectory length: {len(expert_trajectory)}")
        
        # Get future trajectory for comparison  
        future_traj = list(scenario.get_ego_future_trajectory(0, 8.0, 160))  # 8s at 20Hz
        print(f"  - Future trajectory length: {len(future_traj)}")
        
        # Test LogFuturePlanner
        print("\n=== Testing LogFuturePlanner ===")
        log_planner = LogFuturePlanner(
            scenario=scenario,
            num_poses=16,
            future_time_horizon=8.0
        )
        
        # Initialize planner
        initialization = PlannerInitialization(
            route_roadblock_ids=[],
            mission_goal=scenario.get_mission_goal(),
            map_api=scenario.map_api
        )
        log_planner.initialize(initialization)
        
        # Generate CSV data with exact scenario length
        print("\n=== Generating CSV Data with Exact Length ===")
        csv_file_path = "/home/chen/nuplan-devkit/offline_evaluation/data/sample_trajectories.csv"
        
        # Use already extracted expert trajectory data
        num_iterations = scenario.get_number_of_iterations()
        
        print(f"  - Expert trajectory length: {len(expert_trajectory)}")
        print(f"  - Scenario iterations: {num_iterations}")
        print(f"  - Scenario token: {scenario.token}")
        
        # Generate CSV data with exact length matching the scenario
        import pandas as pd
        import numpy as np
        
        csv_data = []
        for i, state in enumerate(expert_trajectory):
            csv_data.append({
                'nuplan_token': scenario.token,
                'nuplan_log': scenario.log_name if hasattr(scenario, 'log_name') else 'expert_log',
                'scenario_id': f'scenario_{scenario.token}',
                'iteration': i,
                'timestamp_us': state.time_point.time_us,
                'ego_x': state.rear_axle.x,
                'ego_y': state.rear_axle.y,
                'ego_heading': state.rear_axle.heading,
                'ego_velocity_x': state.dynamic_car_state.rear_axle_velocity_2d.x,
                'ego_velocity_y': state.dynamic_car_state.rear_axle_velocity_2d.y,
                'ego_acceleration_x': state.dynamic_car_state.rear_axle_acceleration_2d.x,
                'ego_acceleration_y': state.dynamic_car_state.rear_axle_acceleration_2d.y,
                'tire_steering_angle': getattr(state.tire_steering_angle, 'value', 0.0) if hasattr(state, 'tire_steering_angle') else 0.0,
                'scenario_type': 'stationary'
            })
        
        # Save to CSV
        df = pd.DataFrame(csv_data)
        df.to_csv(csv_file_path, index=False)
        print(f"  ✅ Generated CSV with {len(csv_data)} rows matching scenario length")
        print(f"  ✅ Saved to: {csv_file_path}")
        
        # Test CSV planner with new data
        print("\n=== Testing CSVLogFuturePlanner with Exact Length Data ===")
        csv_planner = CSVLogFuturePlanner(
            csv_file_path=csv_file_path,
            scenario=scenario,
            num_poses=16,
            future_time_horizon=8.0
        )
        
        csv_planner.initialize(initialization)
        
        # Simulate a few steps to see trajectory generation
        print("\n=== Trajectory Generation Comparison ===")
        
        time_controller = StepSimulationTimeController(scenario)
        observations = IDMAgents(
            target_velocity=10.0,
            min_gap_to_lead_agent=1.0,
            headway_time=1.5,
            accel_max=1.0,
            decel_max=2.0,
            scenario=scenario,
            open_loop_detections_types=[]
        )
        
        for iteration in range(min(5, scenario.get_number_of_iterations())):
            print(f"\n--- Iteration {iteration} ---")
            
            # Get current input
            current_input = PlannerInput(
                iteration=time_controller.get_iteration(),
                history=scenario.get_past_tracked_objects(
                    time_controller.get_iteration(), 
                    time_horizon=2.0, 
                    num_samples=20
                ),
                traffic_light_data=scenario.get_traffic_light_data_at_iteration(
                    time_controller.get_iteration()
                )
            )
            
            # Test LogFuturePlanner trajectory
            try:
                log_trajectory = log_planner.compute_planner_trajectory(current_input)
                log_states = log_trajectory.get_sampled_trajectory()
                print(f"  LogFuturePlanner trajectory length: {len(log_states)}")
                print(f"  LogFuturePlanner start time: {log_states[0].time_point.time_us}")
                print(f"  LogFuturePlanner end time: {log_states[-1].time_point.time_us}")
                print(f"  LogFuturePlanner duration: {(log_states[-1].time_point.time_us - log_states[0].time_point.time_us) / 1e6:.2f}s")
            except Exception as e:
                print(f"  LogFuturePlanner error: {e}")
            
            # Test CSVLogFuturePlanner trajectory  
            try:
                csv_trajectory = csv_planner.compute_planner_trajectory(current_input)
                csv_states = csv_trajectory.get_sampled_trajectory()
                print(f"  CSVLogFuturePlanner trajectory length: {len(csv_states)}")
                print(f"  CSVLogFuturePlanner start time: {csv_states[0].time_point.time_us}")
                print(f"  CSVLogFuturePlanner end time: {csv_states[-1].time_point.time_us}")
                print(f"  CSVLogFuturePlanner duration: {(csv_states[-1].time_point.time_us - csv_states[0].time_point.time_us) / 1e6:.2f}s")
            except Exception as e:
                print(f"  CSVLogFuturePlanner error: {e}")
            
            # Advance time
            time_controller.next_iteration()
            
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()