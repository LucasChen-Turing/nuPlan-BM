#!/usr/bin/env python3
"""
Simple test script for CSV planner to debug the TimePoint comparison issue.
"""

import sys
from pathlib import Path

# Add nuPlan root to path
nuplan_root = Path(__file__).parent.parent
sys.path.insert(0, str(nuplan_root))
sys.path.insert(0, str(nuplan_root / "offline_evaluation"))

from planners.csv_log_future_planner import CSVLogFuturePlanner
from nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
from nuplan.common.actor_state.state_representation import TimePoint
from nuplan.planning.simulation.planner.abstract_planner import PlannerInput

def test_csv_planner():
    """Test CSV planner with sample data."""
    try:
        # Initialize planner
        csv_file = "/home/chen/nuplan-devkit/offline_evaluation/data/sample_trajectories.csv"
        planner = CSVLogFuturePlanner(
            csv_file_path=csv_file,
            num_poses=16,
            future_time_horizon=8.0
        )
        
        print(f"Loaded planner with {len(planner.get_available_scenarios())} scenarios")
        
        # Test with first scenario
        scenarios = planner.get_available_scenarios()
        test_scenario = scenarios[0]
        print(f"Testing scenario: {test_scenario}")
        
        planner.set_current_scenario(test_scenario)
        
        # Create test input
        iteration = SimulationIteration(index=0, time_point=TimePoint(0))
        planner_input = PlannerInput(
            iteration=iteration,
            history=None,
            traffic_light_data=None
        )
        
        print(f"Testing with iteration index: {iteration.index}")
        print(f"Testing with time point: {iteration.time_point}")
        
        # This should trigger the comparison issue
        trajectory = planner.compute_planner_trajectory(planner_input)
        
        print(f"Success! Generated trajectory with {len(trajectory.get_sampled_trajectory())} points")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_csv_planner()