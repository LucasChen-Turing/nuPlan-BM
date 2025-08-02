"""
Debug script to test planner-expert metrics with minimal example.
"""

import sys
sys.path.append('/home/chen/nuplan-devkit')

from mock_simulation_history import create_mock_simulation_history
from nuplan.planning.metrics.evaluation_metrics.common.planner_expert_average_l2_error_within_bound import (
    PlannerExpertAverageL2ErrorStatistics
)

def debug_metric():
    print("🔍 Creating minimal test case...")
    
    # Create minimal simulation history
    history, scenario = create_mock_simulation_history(
        num_samples=20,  # Only 20 samples (2 seconds)
        dt=0.1,
        trajectory_noise=0.1
    )
    
    print(f"   History samples: {len(history.data)}")
    print(f"   Expert trajectory: {len(scenario.get_expert_ego_trajectory())}")
    print(f"   Database interval: {scenario.database_interval}")
    
    # Test one simple metric
    metric = PlannerExpertAverageL2ErrorStatistics(
        name="debug_metric",
        category="test", 
        comparison_horizon=[2, 3],  # Just 2 simple horizons
        comparison_frequency=1,     # 1 Hz sampling
        max_average_l2_error_threshold=1.0
    )
    
    print("\n🧪 Testing metric computation...")
    try:
        results = metric.compute(history, scenario)
        print(f"   ✅ Success! Results: {len(results)} metric statistics")
        for result in results:
            print(f"      Score: {result.metric_score}")
    except Exception as e:
        print(f"   ❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_metric()