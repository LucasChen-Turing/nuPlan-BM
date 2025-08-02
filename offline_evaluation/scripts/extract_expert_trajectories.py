#!/usr/bin/env python3
"""
Extract Expert Trajectories from nuPlan Database

This script extracts real expert (ground truth) trajectories from nuPlan database scenarios
and saves them in CSV format for offline evaluation. This ensures the CSV trajectories
match the actual expert trajectories that nuPlan will compare against during evaluation.
"""

import argparse
import pandas as pd
from pathlib import Path
from typing import List, Dict, Any
import logging

from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
from nuplan.planning.utils.multithreading.worker_sequential import Sequential
from nuplan.database.nuplan_db_orm.nuplandb_wrapper import NuPlanDBWrapper
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_utils import ScenarioMapping

logger = logging.getLogger(__name__)


def extract_scenario_trajectories(
    db_files: List[str],
    scenario_types: List[str] = ["low_magnitude_speed"],
    limit_scenarios: int = 5,
    trajectory_length: int = 32,  # 8 seconds at 4Hz
    output_file: str = "expert_trajectories.csv"
) -> None:
    """
    Extract expert trajectories from nuPlan database scenarios.
    
    :param db_files: List of database file paths
    :param scenario_types: List of scenario types to extract
    :param limit_scenarios: Maximum number of scenarios to extract
    :param trajectory_length: Number of trajectory points to extract (32 = 8 seconds)
    :param output_file: Output CSV file path
    """
    
    # Initialize scenario builder
    scenario_builder = NuPlanScenarioBuilder(
        data_root="/home/chen/nuplan-devkit/data",
        map_root="/home/chen/nuplan-devkit/maps", 
        sensor_root="/home/chen/nuplan-devkit/data",
        db_files=db_files,
        map_version="nuplan-maps-v1.0",
        verbose=True
    )
    
    # Create scenario filter
    scenario_filter = ScenarioFilter(
        scenario_types=scenario_types,
        scenario_tokens=None,
        log_names=None,
        map_names=None,
        num_scenarios_per_type=limit_scenarios,
        limit_total_scenarios=limit_scenarios,
        timestamp_threshold_s=None,
        ego_displacement_minimum_m=None,
        ego_start_speed_threshold=None,
        ego_stop_speed_threshold=None,
        speed_noise_tolerance=None,
        expand_scenarios=False,
        remove_invalid_goals=True,
        shuffle=False
    )
    
    # Get scenarios
    worker = Sequential()
    scenarios = scenario_builder.get_scenarios(scenario_filter, worker)
    
    logger.info(f"Found {len(scenarios)} scenarios to extract")
    
    trajectories_data = []
    
    for scenario_idx, scenario in enumerate(scenarios):
        try:
            scenario_id = f"scenario_{scenario_idx:03d}"
            token = scenario.token
            log_name = scenario.log_name
            
            logger.info(f"Extracting trajectory from scenario {scenario_id} (token: {token})")
            
            # Get scenario duration and limit to our trajectory length
            total_iterations = min(scenario.get_number_of_iterations(), trajectory_length)
            
            for iteration in range(total_iterations):
                try:
                    # Get ego state at this iteration
                    ego_state = scenario.get_ego_state_at_iteration(iteration)
                    
                    # Extract pose information
                    pose = ego_state.car_footprint.oriented_box.center
                    x = pose.x
                    y = pose.y
                    heading = pose.heading
                    
                    # Extract dynamic state
                    dynamic_state = ego_state.dynamic_car_state
                    velocity_2d = dynamic_state.rear_axle_velocity_2d
                    acceleration_2d = dynamic_state.rear_axle_acceleration_2d
                    
                    vx = velocity_2d.x
                    vy = velocity_2d.y
                    ax = acceleration_2d.x
                    ay = acceleration_2d.y
                    
                    angular_velocity = dynamic_state.angular_velocity
                    angular_acceleration = dynamic_state.angular_acceleration
                    tire_steering_angle = ego_state.tire_steering_angle
                    
                    # Get timestamp
                    timestamp_us = ego_state.time_point.time_us
                    
                    # Add to trajectories data
                    trajectory_row = {
                        'scenario_id': scenario_id,
                        'iteration': iteration,
                        'timestamp_us': timestamp_us,
                        'ego_x': float(x),
                        'ego_y': float(y),
                        'ego_heading': float(heading),
                        'ego_velocity_x': float(vx),
                        'ego_velocity_y': float(vy),
                        'ego_acceleration_x': float(ax),
                        'ego_acceleration_y': float(ay),
                        'ego_angular_velocity': float(angular_velocity),
                        'ego_angular_acceleration': float(angular_acceleration),
                        'tire_steering_angle': float(tire_steering_angle),
                        'scenario_type': scenario_types[0] if scenario_types else 'unknown',
                        'nuplan_token': token,
                        'nuplan_log': log_name
                    }
                    
                    trajectories_data.append(trajectory_row)
                    
                except Exception as e:
                    logger.warning(f"Failed to extract iteration {iteration} from scenario {scenario_id}: {e}")
                    continue
                    
        except Exception as e:
            logger.error(f"Failed to process scenario {scenario_idx}: {e}")
            continue
    
    # Save to CSV
    if trajectories_data:
        df = pd.DataFrame(trajectories_data)
        output_path = Path(output_file)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        df.to_csv(output_path, index=False)
        logger.info(f"Saved {len(trajectories_data)} trajectory points from {len(scenarios)} scenarios to {output_path}")
        
        # Print summary
        scenarios_summary = df.groupby('scenario_id').size()
        logger.info(f"Scenarios extracted: {len(scenarios_summary)}")
        logger.info(f"Points per scenario: {scenarios_summary.describe()}")
        
    else:
        logger.error("No trajectory data extracted!")


def main():
    parser = argparse.ArgumentParser(description="Extract expert trajectories from nuPlan database")
    parser.add_argument(
        "--db-files", 
        nargs="+",
        default=["/home/chen/nuplan-devkit/data/cache/mini/2021.05.12.22.00.38_veh-35_01008_01518.db"],
        help="Database files to extract from"
    )
    parser.add_argument(
        "--scenario-types",
        nargs="+", 
        default=["low_magnitude_speed"],
        help="Scenario types to extract"
    )
    parser.add_argument(
        "--limit-scenarios",
        type=int,
        default=5,
        help="Maximum number of scenarios to extract"
    )
    parser.add_argument(
        "--trajectory-length",
        type=int,
        default=32,
        help="Number of trajectory points to extract (32 = 8 seconds at 4Hz)"
    )
    parser.add_argument(
        "--output-file",
        default="/home/chen/nuplan-devkit/offline_evaluation/data/expert_trajectories.csv",
        help="Output CSV file path"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose logging"
    )
    
    args = parser.parse_args()
    
    # Set up logging
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Extract trajectories
    extract_scenario_trajectories(
        db_files=args.db_files,
        scenario_types=args.scenario_types,
        limit_scenarios=args.limit_scenarios,
        trajectory_length=args.trajectory_length,
        output_file=args.output_file
    )


if __name__ == "__main__":
    main()