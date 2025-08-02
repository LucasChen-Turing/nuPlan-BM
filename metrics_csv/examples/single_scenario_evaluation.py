#!/usr/bin/env python3
"""
Test Single Scenario with CSV Data

Simple test to load one real nuPlan scenario and test if CSV data 
works with the MetricsEngine.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import sys
import pandas as pd
from pathlib import Path
from unittest.mock import Mock

# Add nuplan to path
sys.path.append('/home/chen/nuplan-devkit')

# nuPlan imports
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
from nuplan.planning.utils.multithreading.worker_sequential import Sequential
from nuplan.planning.simulation.history.simulation_history import SimulationHistory, SimulationHistorySample
from nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_average_l2_error_within_bound import PlannerExpertAverageL2ErrorStatistics
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_average_heading_error_within_bound import PlannerExpertAverageHeadingErrorStatistics
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_final_l2_error_within_bound import PlannerExpertFinalL2ErrorStatistics
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_final_heading_error_within_bound import PlannerExpertFinalHeadingErrorStatistics
from nuplan.planning.metrics.evaluation_metrics.common.planner_miss_rate_within_bound import PlannerMissRateStatistics
from nuplan.planning.metrics.metric_engine import MetricsEngine
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.car_footprint import CarFootprint
from nuplan.common.actor_state.dynamic_car_state import DynamicCarState
from nuplan.common.actor_state.state_representation import StateSE2, TimePoint
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters

# Local imports
sys.path.append('/home/chen/nuplan-devkit/metrics_csv')
from src.planning.csv_trajectory import CSVTrajectory, CSVTrajectoryPoint


def load_one_scenario():
    """Load exactly one real scenario from the database."""
    print("🗂️  Loading one real nuPlan scenario...")
    
    # Try different data directories
    possible_data_roots = [
        "/home/chen/nuplan-devkit/data/cache/mini",
        "/home/chen/nuplan-devkit/data/cache/mini_working",
        "/home/chen/nuplan-devkit/data"
    ]
    
    for data_root in possible_data_roots:
        try:
            print(f"   Trying data root: {data_root}")
            
            scenario_builder = NuPlanScenarioBuilder(
                data_root=data_root,
                map_root="/home/chen/nuplan-devkit/maps",
                sensor_root=None,
                db_files=None,
                map_version="nuplan-maps-v1.0"
            )
            
            # Very simple filter - just get ANY one scenario
            scenario_filter = ScenarioFilter(
                scenario_types=None,
                scenario_tokens=None,
                log_names=None,
                map_names=None,
                num_scenarios_per_type=None,
                limit_total_scenarios=1,
                timestamp_threshold_s=5.0,  # Very short minimum
                ego_displacement_minimum_m=None,
                expand_scenarios=False,
                remove_invalid_goals=False,  # Don't be picky
                shuffle=False
            )
            
            scenarios = scenario_builder.get_scenarios(scenario_filter, Sequential())
            
            if scenarios:
                scenario = scenarios[0]
                print(f"   ✅ Successfully loaded scenario: {scenario.scenario_name}")
                print(f"      Type: {scenario.scenario_type}")
                print(f"      Duration: {scenario.get_number_of_iterations()} iterations")
                return scenario
                
        except Exception as e:
            print(f"   ❌ Failed with {data_root}: {e}")
            continue
    
    print(f"   ❌ Could not load any scenario")
    return None


def csv_row_to_ego_state(row, vehicle_params):
    """Convert CSV row to nuPlan EgoState."""
    timestamp_s = float(row['timestamp_us']) / 1e6
    
    # Create rear axle pose
    rear_axle_pose = StateSE2(
        x=float(row['ego_x']),
        y=float(row['ego_y']),
        heading=float(row['ego_heading'])
    )
    
    # Create dynamic state
    dynamic_state = DynamicCarState(
        rear_axle_to_center_dist=vehicle_params.rear_axle_to_center,
        rear_axle_velocity_2d=StateSE2(
            x=float(row['ego_velocity_x']),
            y=float(row['ego_velocity_y']),
            heading=0.0
        ),
        rear_axle_acceleration_2d=StateSE2(
            x=float(row['ego_acceleration_x']),
            y=float(row['ego_acceleration_y']),
            heading=0.0
        ),
        angular_velocity=0.0,
        angular_acceleration=0.0
    )
    
    # Create car footprint
    car_footprint = CarFootprint.build_from_rear_axle(
        rear_axle_pose=rear_axle_pose,
        vehicle_parameters=vehicle_params
    )
    
    # Create time point
    time_point = TimePoint(int(timestamp_s * 1e6))
    
    return EgoState(
        car_footprint=car_footprint,
        dynamic_car_state=dynamic_state,
        tire_steering_angle=float(row['tire_steering_angle']),
        is_in_auto_mode=True,
        time_point=time_point
    )


def align_csv_timestamps_to_expert(csv_data, expert_trajectory):
    """Align CSV timestamps to match expert trajectory timing."""
    print("🕐 Aligning CSV timestamps to expert trajectory...")
    
    # Get expert trajectory time range
    expert_states = list(expert_trajectory)
    expert_start_time = expert_states[0].time_point.time_us
    expert_end_time = expert_states[-1].time_point.time_us
    expert_duration = expert_end_time - expert_start_time
    
    print(f"   Expert trajectory: {expert_start_time} to {expert_end_time} ({expert_duration/1e6:.2f}s)")
    
    # Get CSV time range
    csv_start_time = csv_data['timestamp_us'].min()
    csv_end_time = csv_data['timestamp_us'].max()
    csv_duration = csv_end_time - csv_start_time
    
    print(f"   CSV data: {csv_start_time} to {csv_end_time} ({csv_duration/1e6:.2f}s)")
    
    # Create linear mapping from CSV time range to expert time range
    # We'll map CSV data to use only the first portion of expert trajectory
    # to ensure we stay within bounds
    expert_portion = min(csv_duration, expert_duration * 0.8)  # Use 80% of expert duration max
    expert_target_end = expert_start_time + expert_portion
    
    print(f"   Mapping CSV to: {expert_start_time} to {expert_target_end} ({expert_portion/1e6:.2f}s)")
    
    # Apply linear transformation: new_time = a * old_time + b
    if csv_duration > 0:
        scale_factor = expert_portion / csv_duration
        offset = expert_start_time - (csv_start_time * scale_factor)
        
        csv_data = csv_data.copy()
        csv_data['timestamp_us'] = csv_data['timestamp_us'] * scale_factor + offset
        
        print(f"   ✅ Applied scaling: factor={scale_factor:.6f}, offset={offset:.0f}")
        print(f"   New CSV range: {csv_data['timestamp_us'].min():.0f} to {csv_data['timestamp_us'].max():.0f}")
    
    return csv_data


def create_csv_simulation_history(csv_file, scenario_id, real_scenario):
    """Create SimulationHistory from CSV data aligned to real scenario timing."""
    print(f"🔄 Creating aligned SimulationHistory from CSV for {scenario_id}...")
    
    df = pd.read_csv(csv_file)
    scenario_data = df[df['scenario_id'] == scenario_id].copy()
    scenario_data = scenario_data.sort_values('planning_iteration')
    
    print(f"   Found {len(scenario_data)} timesteps for {scenario_id}")
    
    if len(scenario_data) == 0:
        print(f"   ❌ No data found for scenario {scenario_id}")
        return None
    
    # Get expert trajectory for timestamp alignment
    expert_trajectory = real_scenario.get_expert_ego_trajectory()
    
    # Align CSV timestamps to expert trajectory
    scenario_data = align_csv_timestamps_to_expert(scenario_data, expert_trajectory)
    
    # Create simulation history with real scenario's map and mission
    history = SimulationHistory(real_scenario.map_api, real_scenario.get_mission_goal())
    vehicle_params = get_pacifica_parameters()
    
    for idx, row in scenario_data.iterrows():
        # Convert CSV to ego state with aligned timestamps
        ego_state = csv_row_to_ego_state(row, vehicle_params)
        
        # Create multi-point trajectory starting with current state
        trajectory_points = []
        base_time = float(row['timestamp_us']) / 1e6
        
        # ALWAYS start with current ego state at current time
        current_point = CSVTrajectoryPoint(
            timestamp=base_time,
            x=float(row['ego_x']),
            y=float(row['ego_y']),
            heading=float(row['ego_heading']),
            velocity_x=float(row['ego_velocity_x']),
            velocity_y=float(row['ego_velocity_y'])
        )
        trajectory_points.append(current_point)
        
        try:
            # Add horizon data as future trajectory points
            import json
            horizon_x = json.loads(row['horizon_x'])
            horizon_y = json.loads(row['horizon_y'])  
            horizon_heading = json.loads(row['horizon_heading'])
            horizon_vx = json.loads(row['horizon_velocity_x'])
            horizon_vy = json.loads(row['horizon_velocity_y'])
            horizon_seconds = float(row['horizon_seconds'])
            
            # Add future points from horizon
            if len(horizon_x) > 0:
                dt = horizon_seconds / len(horizon_x)
                for i in range(len(horizon_x)):
                    # Future trajectory points
                    future_time = base_time + (i + 1) * dt
                    point = CSVTrajectoryPoint(
                        timestamp=future_time,
                        x=horizon_x[i],
                        y=horizon_y[i], 
                        heading=horizon_heading[i],
                        velocity_x=horizon_vx[i],
                        velocity_y=horizon_vy[i]
                    )
                    trajectory_points.append(point)
            
        except Exception as e:
            print(f"   ⚠️  Failed to parse horizon for row {idx}: {e}")
            # If horizon parsing fails, just use current state (already added above)
        
        trajectory = CSVTrajectory(trajectory_points)
        
        # Debug: print trajectory time range
        if idx < 3:  # Only for first few samples
            traj_times = [p.timestamp for p in trajectory_points]
            print(f"   Trajectory {idx}: {min(traj_times):.6f} to {max(traj_times):.6f} ({len(trajectory_points)} points)")
        
        # Create simulation iteration
        iteration = SimulationIteration(
            time_point=ego_state.time_point,
            index=int(row['planning_iteration'])
        )
        
        # Create simulation sample
        sample = SimulationHistorySample(
            iteration=iteration,
            ego_state=ego_state,
            trajectory=trajectory,
            observation=Mock(),
            traffic_light_status=[]
        )
        
        history.add_sample(sample)
    
    print(f"   ✅ Created aligned SimulationHistory with {len(history.data)} samples")
    return history


def test_csv_with_metrics_engine():
    """Test CSV data with real nuPlan MetricsEngine."""
    print("🚀 Testing CSV Data with Real nuPlan MetricsEngine")
    print("=" * 60)
    
    # Load one real scenario
    real_scenario = load_one_scenario()
    if not real_scenario:
        print("❌ Cannot proceed without a real scenario")
        return
    
    # Load CSV data
    csv_file = "/home/chen/nuplan-devkit/metrics_csv/data/sample_csv/test_planner_data.csv"
    if not Path(csv_file).exists():
        print(f"❌ CSV file not found: {csv_file}")
        return
    
    # Get first scenario from CSV
    df = pd.read_csv(csv_file)
    csv_scenario_id = df['scenario_id'].iloc[0]
    print(f"📊 Using CSV scenario: {csv_scenario_id}")
    
    # Create simulation history from CSV with timestamp alignment
    csv_history = create_csv_simulation_history(csv_file, csv_scenario_id, real_scenario)
    if not csv_history:
        return
    
    # Create ALL 5 metrics as expected
    print("📊 Setting up ALL 5 metrics...")
    
    # Base L2 error metric
    l2_metric = PlannerExpertAverageL2ErrorStatistics(
        name="planner_expert_average_l2_error",
        category="accuracy", 
        comparison_horizon=[1],
        comparison_frequency=1,
        max_average_l2_error_threshold=2.0
    )
    
    # All 5 metrics with dependencies
    all_metrics = [
        l2_metric,
        PlannerExpertAverageHeadingErrorStatistics(
            name="planner_expert_average_heading_error",
            category="accuracy",
            planner_expert_average_l2_error_within_bound_metric=l2_metric,
            max_average_heading_error_threshold=0.3
        ),
        PlannerExpertFinalL2ErrorStatistics(
            name="planner_expert_final_l2_error", 
            category="accuracy",
            planner_expert_average_l2_error_within_bound_metric=l2_metric,
            max_final_l2_error_threshold=3.0
        ),
        PlannerExpertFinalHeadingErrorStatistics(
            name="planner_expert_final_heading_error",
            category="accuracy",
            planner_expert_average_l2_error_within_bound_metric=l2_metric,
            max_final_heading_error_threshold=0.5
        ),
        PlannerMissRateStatistics(
            name="planner_miss_rate",
            category="accuracy", 
            planner_expert_average_l2_error_within_bound_metric=l2_metric,
            max_displacement_threshold=[1.0],
            max_miss_rate_threshold=0.2
        )
    ]
    
    print(f"   ✅ Configured {len(all_metrics)} metrics:")
    for metric in all_metrics:
        print(f"      - {metric.name}")
    
    # Create metrics engine
    save_path = Path("/home/chen/nuplan-devkit/metrics_csv/results/single_test")
    metrics_engine = MetricsEngine(main_save_path=save_path, metrics=all_metrics)
    
    print("🧮 Running MetricsEngine...")
    try:
        # Run metrics computation
        computed_metrics = metrics_engine.compute(
            history=csv_history,
            scenario=real_scenario,
            planner_name="csv_planner"
        )
        
        print(f"✅ SUCCESS! MetricsEngine completed")
        print(f"📊 Generated metrics: {list(computed_metrics.keys())}")
        
        # Create JSON results similar to planner_expert_metrics_results.json
        json_results = {}
        
        # Process and display actual metric results
        for metric_name, metric_files in computed_metrics.items():
            print(f"\n📈 {metric_name}: {len(metric_files)} files generated")
            
            # Read and display metric results from MetricFile
            for metric_file in metric_files:
                try:
                    print(f"   📊 Scenario: {metric_file.key.scenario_name} ({metric_file.key.scenario_type})")
                    print(f"   🎯 Planner: {metric_file.key.planner_name}")
                    
                    # Process each metric statistic in the file
                    for metric_result in metric_file.metric_statistics:
                        print(f"\n   📈 Metric: {metric_result.metric_computator}")
                        print(f"   🎯 **SCORE: {metric_result.metric_score:.4f}**")
                        print(f"   📂 Category: {metric_result.metric_category}")
                        
                        # Convert to JSON format
                        metric_json = {
                            "metric_score": float(metric_result.metric_score),
                            "category": metric_result.metric_category,
                            "statistics": []
                        }
                        
                        # Add statistics
                        for stat in metric_result.statistics:
                            stat_json = {
                                "name": stat.name,
                                "value": float(stat.value),
                                "unit": stat.unit,
                                "type": stat.type.name
                            }
                            metric_json["statistics"].append(stat_json)
                            
                            if stat.type.name in ['MEAN', 'BOOLEAN']:  # Focus on key stats
                                print(f"      {stat.name}: {stat.value:.4f} {stat.unit}")
                        
                        # Add time series if available
                        if hasattr(metric_result, 'time_series') and metric_result.time_series:
                            ts = metric_result.time_series
                            metric_json["time_series"] = {
                                "unit": ts.unit,
                                "timestamps": [int(t) for t in ts.time_stamps],
                                "values": [float(v) for v in ts.values],
                                "num_samples": len(ts.values)
                            }
                            print(f"   📈 Time Series: {len(ts.values)} samples")
                            print(f"      Values: {[f'{v:.3f}' for v in ts.values]}")
                            print(f"      Mean L2 Error: {sum(ts.values)/len(ts.values):.4f}m")
                        
                        # Add to results with metric name as key
                        if metric_result.metric_computator not in json_results:
                            json_results[metric_result.metric_computator] = []
                        json_results[metric_result.metric_computator].append(metric_json)
                        
                except Exception as e:
                    print(f"   ❌ Could not process metric file: {e}")
                    import traceback
                    traceback.print_exc()
        
        # Save JSON results
        import json
        results_file = Path("/home/chen/nuplan-devkit/metrics_csv/results/csv_real_nuplan_metrics.json")
        results_file.parent.mkdir(exist_ok=True)
        
        with open(results_file, 'w') as f:
            json.dump(json_results, f, indent=2)
        
        print(f"\n💾 **JSON Results saved to: {results_file}**")
        print(f"📊 Contains {len(json_results)} metric types with real nuPlan scores")
        
        # Check if files were created
        if save_path.exists():
            files = list(save_path.glob("**/*"))
            print(f"\n💾 Created {len(files)} result files in {save_path}")
            for f in files[:5]:  # Show first 5
                print(f"      {f.name}")
        
        return True
        
    except Exception as e:
        print(f"❌ MetricsEngine failed: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = test_csv_with_metrics_engine()
    print(f"\n🎯 Final Result: {'SUCCESS' if success else 'FAILED'}")