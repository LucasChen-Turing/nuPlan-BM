"""
Real Database Metrics Evaluation System

This module replaces the synthetic trajectory approach with real nuPlan database scenarios,
using actual expert driving logs and simulated planner trajectories for comparison.
"""

import sys
import os
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple
import numpy as np
import json
from glob import glob

# Add nuplan-devkit to path
sys.path.append('/home/chen/nuplan-devkit')

# Core nuPlan imports
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.scenario_filter import ScenarioFilter
from nuplan.planning.scenario_builder.scenario_utils import sample_indices_with_time_horizon
from nuplan.common.maps.nuplan_map.map_factory import get_maps_db
from nuplan.planning.utils.multithreading.worker_sequential import Sequential
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.state_representation import StateSE2, TimePoint, StateVector2D
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters
from nuplan.common.actor_state.dynamic_car_state import DynamicCarState
from nuplan.common.actor_state.car_footprint import CarFootprint

# Metrics imports
from nuplan.planning.metrics.metric_engine import MetricsEngine
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_average_l2_error_within_bound import (
    PlannerExpertAverageL2ErrorStatistics
)
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_average_heading_error_within_bound import (
    PlannerExpertAverageHeadingErrorStatistics
)
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_final_l2_error_within_bound import (
    PlannerExpertFinalL2ErrorStatistics
)
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_final_heading_error_within_bound import (
    PlannerExpertFinalHeadingErrorStatistics
)
from nuplan.planning.metrics.evaluation_metrics.common.planner_miss_rate_within_bound import (
    PlannerMissRateStatistics
)

# Simulation imports
from nuplan.planning.simulation.history.simulation_history import SimulationHistory, SimulationHistorySample
from nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory
from nuplan.planning.simulation.observation.observation_type import Observation


class RealDataTrajectory(AbstractTrajectory):
    """
    Trajectory implementation using real scenario context with simulated planner behavior.
    """
    
    def __init__(self, expert_states: List[EgoState], noise_level: float = 0.1):
        """
        Create planner trajectory based on expert states with realistic noise.
        
        Args:
            expert_states: Real expert trajectory from database
            noise_level: Standard deviation for trajectory noise
        """
        self._expert_states = expert_states
        self._noise_level = noise_level
        self._planner_states = self._generate_planner_trajectory()
        
    def _generate_planner_trajectory(self) -> List[EgoState]:
        """Generate realistic planner trajectory based on expert with noise."""
        planner_states = []
        vehicle_params = get_pacifica_parameters()
        
        for i, expert_state in enumerate(self._expert_states):
            # Add realistic noise to expert trajectory
            expert_pose = expert_state.rear_axle
            
            # Spatial noise (position uncertainty)
            noise_x = np.random.normal(0, self._noise_level)
            noise_y = np.random.normal(0, self._noise_level)
            noise_heading = np.random.normal(0, self._noise_level * 0.1)
            
            # Temporal consistency (smooth noise transitions)
            if i > 0:
                prev_expert = self._expert_states[i-1].rear_axle
                prev_planner = planner_states[-1].rear_axle
                
                # Maintain some correlation with previous errors
                error_x = prev_planner.x - prev_expert.x
                error_y = prev_planner.y - prev_expert.y
                error_heading = prev_planner.heading - prev_expert.heading
                
                # Apply momentum to errors (80% persistence + 20% new noise)
                noise_x = 0.8 * error_x + 0.2 * noise_x
                noise_y = 0.8 * error_y + 0.2 * noise_y
                noise_heading = 0.8 * error_heading + 0.2 * noise_heading
            
            # Create planner pose
            planner_pose = StateSE2(
                expert_pose.x + noise_x,
                expert_pose.y + noise_y,
                expert_pose.heading + noise_heading
            )
            
            # Build ego state with realistic dynamics
            car_footprint = CarFootprint.build_from_rear_axle(planner_pose, vehicle_params)
            
            # Use expert's velocity with slight variation
            expert_velocity = expert_state.dynamic_car_state.rear_axle_velocity_2d
            velocity_noise_x = np.random.normal(0, self._noise_level * 0.5)
            velocity_noise_y = np.random.normal(0, self._noise_level * 0.5)
            
            planner_velocity = StateVector2D(
                expert_velocity.x + velocity_noise_x,
                expert_velocity.y + velocity_noise_y
            )
            
            dynamic_state = DynamicCarState.build_from_rear_axle(
                rear_axle_to_center_dist=car_footprint.rear_axle_to_center_dist,
                rear_axle_velocity_2d=planner_velocity,
                rear_axle_acceleration_2d=expert_state.dynamic_car_state.rear_axle_acceleration_2d,
                angular_velocity=expert_state.dynamic_car_state.angular_velocity,
                angular_acceleration=expert_state.dynamic_car_state.angular_acceleration
            )
            
            planner_state = EgoState(
                car_footprint=car_footprint,
                dynamic_car_state=dynamic_state,
                tire_steering_angle=expert_state.tire_steering_angle,
                is_in_auto_mode=True,
                time_point=expert_state.time_point
            )
            
            planner_states.append(planner_state)
        
        return planner_states
    
    def get_sampled_trajectory(self) -> List[EgoState]:
        """Return the complete planner trajectory."""
        return self._planner_states
        
    def get_state_at_time(self, time_point: TimePoint) -> EgoState:
        """Get planner state at specific time point."""
        # Find closest state by time
        closest_state = min(
            self._planner_states,
            key=lambda s: abs(s.time_point.time_us - time_point.time_us)
        )
        return closest_state
        
    def get_state_at_times(self, time_points: List[TimePoint]) -> List[EgoState]:
        """Get planner states at multiple time points."""
        return [self.get_state_at_time(tp) for tp in time_points]
    
    @property
    def start_time(self) -> TimePoint:
        """Get trajectory start time."""
        return self._planner_states[0].time_point if self._planner_states else TimePoint(0)
        
    @property
    def end_time(self) -> TimePoint:
        """Get trajectory end time."""
        return self._planner_states[-1].time_point if self._planner_states else TimePoint(0)


class RealDatabaseEvaluator:
    """
    Main evaluator class using real nuPlan database scenarios.
    """
    
    def __init__(
        self,
        data_path: str = "/home/chen/nuplan-devkit/data/cache/mini",
        maps_path: str = "/home/chen/nuplan-devkit/maps",
        map_version: str = "nuplan-maps-v1.0"
    ):
        """
        Initialize the real database evaluator.
        
        Args:
            data_path: Path to nuPlan database files
            maps_path: Path to map files
            map_version: Map version to use
        """
        self.data_path = Path(data_path)
        self.maps_path = Path(maps_path)
        self.map_version = map_version
        
        # Find available database files
        self.db_files = list(self.data_path.glob("*.db"))
        print(f"📁 Found {len(self.db_files)} database files")
        
        # Initialize scenario builder
        self.scenario_builder = None
        self._initialize_scenario_builder()
    
    def _initialize_scenario_builder(self):
        """Initialize the nuPlan scenario builder."""
        try:
            print("🔧 Initializing scenario builder...")
            self.scenario_builder = NuPlanScenarioBuilder(
                data_root=str(self.data_path.parent),  # Parent of cache/mini
                map_root=str(self.maps_path),
                sensor_root=str(self.data_path.parent),  # Same as data_root for now
                db_files=[str(db) for db in self.db_files[:3]],  # Use first 3 files for testing
                map_version=self.map_version,
                verbose=True
            )
            print("✅ Scenario builder initialized successfully")
        except Exception as e:
            print(f"❌ Failed to initialize scenario builder: {e}")
            raise
    
    def get_scenarios(
        self,
        scenario_types: Optional[List[str]] = None,
        limit: int = 10,
        min_duration: float = 5.0
    ) -> List:
        """
        Get filtered scenarios from the database.
        
        Args:
            scenario_types: List of scenario types to filter
            limit: Maximum number of scenarios to return
            min_duration: Minimum scenario duration in seconds
            
        Returns:
            List of nuPlan scenarios
        """
        # If no scenario types specified, don't filter by type
        filter_by_type = scenario_types is not None
        
        try:
            print(f"🔍 Filtering scenarios: {scenario_types}")
            
            scenario_filter = ScenarioFilter(
                scenario_types=scenario_types if filter_by_type else None,
                scenario_tokens=None,
                log_names=None,
                map_names=None,
                num_scenarios_per_type=None,
                limit_total_scenarios=limit,
                timestamp_threshold_s=min_duration,  # In seconds
                ego_displacement_minimum_m=None,
                expand_scenarios=False,
                remove_invalid_goals=True,
                shuffle=True
            )
            
            # Create sequential worker for processing
            worker = Sequential()
            scenarios = self.scenario_builder.get_scenarios(scenario_filter, worker)
            print(f"✅ Found {len(scenarios)} scenarios")
            
            return scenarios
            
        except Exception as e:
            print(f"❌ Failed to get scenarios: {e}")
            raise
    
    def create_simulation_history_from_scenario(
        self,
        scenario,
        trajectory_noise: float = 0.1,
        planning_horizon: int = 30
    ) -> Tuple[SimulationHistory, Any]:
        """
        Create SimulationHistory from a real nuPlan scenario.
        
        Args:
            scenario: NuPlan scenario object
            trajectory_noise: Noise level for planner trajectories
            planning_horizon: Planning horizon in timesteps
            
        Returns:
            Tuple of (SimulationHistory, scenario)
        """
        print(f"🚗 Processing scenario: {scenario.scenario_name}")
        print(f"   Type: {scenario.scenario_type}")
        print(f"   Duration: {scenario.get_number_of_iterations() * scenario.database_interval:.1f}s")
        
        # Get real expert trajectory and convert generator to list
        expert_trajectory = list(scenario.get_expert_ego_trajectory())
        print(f"   Expert states: {len(expert_trajectory)}")
        
        # Create a single planner trajectory for the entire scenario duration
        # This ensures consistent length comparison with expert trajectory
        full_planner_trajectory = RealDataTrajectory(
            expert_states=expert_trajectory,
            noise_level=trajectory_noise
        )
        planner_states = full_planner_trajectory.get_sampled_trajectory()
        print(f"   Planner states: {len(planner_states)}")
        
        # Create simulation history
        history = SimulationHistory(scenario.map_api, scenario.get_mission_goal())
        
        # Process each timestep with consistent trajectory reference
        for i in range(len(expert_trajectory)):
            current_ego_state = expert_trajectory[i]
            current_planner_state = planner_states[i]
            
            # Create a trajectory that represents planner's plan from this timestep
            # For metrics, we need to provide a trajectory, but the key comparison
            # happens between the ego states (current_planner_state) and expert states
            remaining_planner_states = planner_states[i:]
            
            if len(remaining_planner_states) > 1:
                timestep_trajectory = RealDataTrajectory(
                    expert_states=remaining_planner_states,
                    noise_level=0.0  # No additional noise, already applied
                )
            else:
                # For final timestep
                timestep_trajectory = RealDataTrajectory(
                    expert_states=[current_planner_state],
                    noise_level=0.0
                )
            
            # Create simulation iteration
            iteration = SimulationIteration(
                time_point=current_ego_state.time_point,
                index=i
            )
            
            # Create observation (simplified for metrics evaluation)
            observation = scenario.get_tracked_objects_at_iteration(i)
            traffic_lights = scenario.get_traffic_light_status_at_iteration(i)
            
            # Create simulation sample - key: use planner state as ego_state
            sample = SimulationHistorySample(
                iteration=iteration,
                ego_state=current_planner_state,  # This is the planner's position
                trajectory=timestep_trajectory,
                observation=observation,
                traffic_light_status=traffic_lights
            )
            
            history.add_sample(sample)
        
        print(f"✅ Created history with {len(history.data)} samples")
        return history, scenario
    
    def setup_metrics(self) -> List:
        """Setup the planner-expert comparison metrics."""
        print("📊 Setting up metrics...")
        
        # Configuration parameters (simplified to avoid indexing issues)
        comparison_horizon = [1]        # Single timestep horizon for stability
        comparison_frequency = 1        # 1 Hz sampling
        
        # Thresholds
        max_average_l2_error_threshold = 2.0
        max_average_heading_error_threshold = 0.3
        max_final_l2_error_threshold = 3.0
        max_final_heading_error_threshold = 0.5
        max_displacement_threshold = [1.0]  # Single threshold to match single horizon
        max_miss_rate_threshold = 0.2
        
        # Create metrics
        base_metric = PlannerExpertAverageL2ErrorStatistics(
            name="planner_expert_average_l2_error",
            category="accuracy",
            comparison_horizon=comparison_horizon,
            comparison_frequency=comparison_frequency,
            max_average_l2_error_threshold=max_average_l2_error_threshold,
            metric_score_unit="score"
        )
        
        metrics = [
            base_metric,
            PlannerExpertAverageHeadingErrorStatistics(
                name="planner_expert_average_heading_error",
                category="accuracy",
                planner_expert_average_l2_error_within_bound_metric=base_metric,
                max_average_heading_error_threshold=max_average_heading_error_threshold,
                metric_score_unit="score"
            ),
            PlannerExpertFinalL2ErrorStatistics(
                name="planner_expert_final_l2_error",
                category="accuracy",
                planner_expert_average_l2_error_within_bound_metric=base_metric,
                max_final_l2_error_threshold=max_final_l2_error_threshold,
                metric_score_unit="score"
            ),
            PlannerExpertFinalHeadingErrorStatistics(
                name="planner_expert_final_heading_error",
                category="accuracy",
                planner_expert_average_l2_error_within_bound_metric=base_metric,
                max_final_heading_error_threshold=max_final_heading_error_threshold,
                metric_score_unit="score"
            ),
            PlannerMissRateStatistics(
                name="planner_miss_rate",
                category="accuracy",
                planner_expert_average_l2_error_within_bound_metric=base_metric,
                max_displacement_threshold=max_displacement_threshold,
                max_miss_rate_threshold=max_miss_rate_threshold,
                metric_score_unit="score"
            )
        ]
        
        print(f"✅ Configured {len(metrics)} metrics")
        return metrics
    
    def evaluate_scenario(self, scenario, output_dir: Path) -> Dict[str, Any]:
        """
        Evaluate a single scenario.
        
        Args:
            scenario: NuPlan scenario to evaluate
            output_dir: Directory to save results
            
        Returns:
            Dictionary of metric results
        """
        try:
            # Create simulation history from real scenario
            history, scenario_obj = self.create_simulation_history_from_scenario(scenario)
            
            # Setup metrics
            metrics = self.setup_metrics()
            
            # Create metrics engine
            engine = MetricsEngine(main_save_path=output_dir, metrics=metrics)
            
            # Compute metrics
            print("🧮 Computing metrics...")
            metric_results = engine.compute_metric_results(history, scenario_obj)
            
            return metric_results
            
        except Exception as e:
            print(f"❌ Error evaluating scenario: {e}")
            raise


def main():
    """Main evaluation function using real nuPlan database."""
    print("🚀 Starting Real Database Metrics Evaluation")
    print("=" * 60)
    
    # Create output directory
    output_dir = Path("/home/chen/nuplan-devkit/metrics_csv/real_results")
    output_dir.mkdir(exist_ok=True)
    
    try:
        # Initialize evaluator
        print("1️⃣ Initializing real database evaluator...")
        evaluator = RealDatabaseEvaluator()
        
        # Get scenarios - first try to find any scenarios without type filtering
        print("\n2️⃣ Loading real scenarios...")
        scenarios = evaluator.get_scenarios(
            scenario_types=None,  # Accept any scenario type
            limit=5,  # Test with just 5 scenarios first
            min_duration=2.0  # At least 2 seconds
        )
        
        if not scenarios:
            print("❌ No scenarios found! Check database files and scenario types.")
            return
        
        # Evaluate scenarios
        print("\n3️⃣ Evaluating scenarios...")
        all_results = []
        
        for i, scenario in enumerate(scenarios):
            print(f"\n--- Scenario {i+1}/{len(scenarios)} ---")
            
            try:
                results = evaluator.evaluate_scenario(scenario, output_dir)
                all_results.append({
                    "scenario_name": scenario.scenario_name,
                    "scenario_type": scenario.scenario_type, 
                    "results": results
                })
                
                # Print summary for this scenario
                print("📈 Scenario Results:")
                for metric_name, stats_list in results.items():
                    for stats in stats_list:
                        print(f"   {metric_name}: {stats.metric_score:.4f}")
                        
            except Exception as e:
                print(f"❌ Failed to evaluate scenario {scenario.scenario_name}: {e}")
                continue
        
        # Save combined results
        if all_results:
            print("\n4️⃣ Saving results...")
            results_file = output_dir / "real_database_evaluation_results.json"
            
            # Convert results to JSON-serializable format
            json_results = []
            for result in all_results:
                json_result = {
                    "scenario_name": result["scenario_name"],
                    "scenario_type": result["scenario_type"],
                    "metrics": {}
                }
                
                for metric_name, stats_list in result["results"].items():
                    json_result["metrics"][metric_name] = {
                        "score": stats_list[0].metric_score if stats_list else 0.0,
                        "category": stats_list[0].metric_category if stats_list else "unknown"
                    }
                
                json_results.append(json_result)
            
            with open(results_file, 'w') as f:
                json.dump(json_results, f, indent=2)
            
            print(f"💾 Results saved to: {results_file}")
            
            # Print summary
            print("\n📊 EVALUATION SUMMARY")
            print("=" * 40)
            
            if json_results:
                total_scenarios = len(json_results)
                print(f"Scenarios evaluated: {total_scenarios}")
                
                # Calculate average scores across scenarios
                metric_names = list(json_results[0]["metrics"].keys())
                for metric_name in metric_names:
                    scores = [r["metrics"][metric_name]["score"] for r in json_results]
                    avg_score = np.mean(scores)
                    print(f"{metric_name}: {avg_score:.4f}")
                
                overall_avg = np.mean([
                    np.mean([r["metrics"][m]["score"] for m in metric_names])
                    for r in json_results
                ])
                print(f"\nOverall Average: {overall_avg:.4f}")
                
                if overall_avg > 0.8:
                    print("🎉 Excellent performance with real data!")
                elif overall_avg > 0.6:
                    print("👍 Good performance with real data")
                else:
                    print("⚠️ Performance needs improvement")
        
        print(f"\n✅ Real database evaluation complete!")
        print(f"📁 Check results in: {output_dir}")
        
    except Exception as e:
        print(f"❌ Evaluation failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()