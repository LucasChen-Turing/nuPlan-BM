"""
Evaluate planner-expert comparison metrics using mock SimulationHistory data.

This script demonstrates how to:
1. Create mock SimulationHistory with planner and expert trajectories
2. Evaluate the 5 planner-expert comparison metrics
3. Display and analyze results
"""

import sys
import os
from pathlib import Path
from typing import List, Dict, Any
import numpy as np
import json

# Add the nuplan-devkit to path
sys.path.append('/home/chen/nuplan-devkit')

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
from nuplan.planning.metrics.metric_result import MetricStatistics

from mock_simulation_history import create_mock_simulation_history


def setup_planner_expert_metrics() -> List:
    """
    Setup the 5 planner-expert comparison metrics.
    
    Returns:
        List of configured metric instances
    """
    # Configuration parameters  
    comparison_horizon = [3, 5, 8]  # 3, 5, 8 steps (0.3s, 0.5s, 0.8s at 10Hz)
    comparison_frequency = 1  # 1 Hz sampling for metric calculation
    
    # Thresholds
    max_average_l2_error_threshold = 2.0  # meters
    max_average_heading_error_threshold = 0.3  # radians (~17 degrees) 
    max_final_l2_error_threshold = 3.0  # meters
    max_final_heading_error_threshold = 0.5  # radians (~29 degrees)
    max_displacement_threshold = [1.0, 1.5, 2.0]  # meters for each horizon
    max_miss_rate_threshold = 0.2  # 20% miss rate
    
    # 1. Base metric: Average L2 Error (this computes all the core data)
    average_l2_error_metric = PlannerExpertAverageL2ErrorStatistics(
        name="planner_expert_average_l2_error",
        category="accuracy",
        comparison_horizon=comparison_horizon,
        comparison_frequency=comparison_frequency,
        max_average_l2_error_threshold=max_average_l2_error_threshold,
        metric_score_unit="score"
    )
    
    # 2. Average Heading Error (depends on average_l2_error_metric)
    average_heading_error_metric = PlannerExpertAverageHeadingErrorStatistics(
        name="planner_expert_average_heading_error",
        category="accuracy",
        planner_expert_average_l2_error_within_bound_metric=average_l2_error_metric,
        max_average_heading_error_threshold=max_average_heading_error_threshold,
        metric_score_unit="score"
    )
    
    # 3. Final L2 Error (depends on average_l2_error_metric)
    final_l2_error_metric = PlannerExpertFinalL2ErrorStatistics(
        name="planner_expert_final_l2_error",
        category="accuracy",
        planner_expert_average_l2_error_within_bound_metric=average_l2_error_metric,
        max_final_l2_error_threshold=max_final_l2_error_threshold,
        metric_score_unit="score"
    )
    
    # 4. Final Heading Error (depends on average_l2_error_metric)
    final_heading_error_metric = PlannerExpertFinalHeadingErrorStatistics(
        name="planner_expert_final_heading_error",
        category="accuracy",
        planner_expert_average_l2_error_within_bound_metric=average_l2_error_metric,
        max_final_heading_error_threshold=max_final_heading_error_threshold,
        metric_score_unit="score"
    )
    
    # 5. Miss Rate (depends on average_l2_error_metric)
    miss_rate_metric = PlannerMissRateStatistics(
        name="planner_miss_rate",
        category="accuracy",
        planner_expert_average_l2_error_within_bound_metric=average_l2_error_metric,
        max_displacement_threshold=max_displacement_threshold,
        max_miss_rate_threshold=max_miss_rate_threshold,
        metric_score_unit="score"
    )
    
    return [
        average_l2_error_metric,
        average_heading_error_metric,
        final_l2_error_metric,
        final_heading_error_metric,
        miss_rate_metric
    ]


def print_metric_results(metric_results: Dict[str, List[MetricStatistics]]) -> None:
    """
    Print metric results in a readable format.
    
    Args:
        metric_results: Dictionary of metric name to list of MetricStatistics
    """
    print("\n" + "="*80)
    print("PLANNER-EXPERT COMPARISON METRICS RESULTS")
    print("="*80)
    
    for metric_name, stats_list in metric_results.items():
        print(f"\n📊 {metric_name.upper()}")
        print("-" * 60)
        
        for stats in stats_list:
            print(f"  Metric Score: {stats.metric_score:.4f}")
            print(f"  Category: {stats.metric_category}")
            
            print(f"  Statistics:")
            for stat in stats.statistics:
                print(f"    • {stat.name}: {stat.value:.4f} {stat.unit} ({stat.type})")
            
            if stats.time_series:
                print(f"  Time Series: {len(stats.time_series.values)} samples")
                print(f"    Unit: {stats.time_series.unit}")
                print(f"    Value range: [{np.min(stats.time_series.values):.4f}, {np.max(stats.time_series.values):.4f}]")


def save_results_to_json(metric_results: Dict[str, List[MetricStatistics]], output_path: Path) -> None:
    """
    Save metric results to JSON file for analysis.
    
    Args:
        metric_results: Dictionary of metric results
        output_path: Path to save JSON file
    """
    results_dict = {}
    
    for metric_name, stats_list in metric_results.items():
        metric_data = []
        for stats in stats_list:
            data = {
                "metric_score": stats.metric_score,
                "category": stats.metric_category,
                "statistics": [
                    {
                        "name": stat.name,
                        "value": float(stat.value),
                        "unit": stat.unit,
                        "type": str(stat.type)
                    }
                    for stat in stats.statistics
                ]
            }
            
            if stats.time_series:
                data["time_series"] = {
                    "unit": stats.time_series.unit,
                    "timestamps": [int(t) for t in stats.time_series.time_stamps],
                    "values": [float(v) for v in stats.time_series.values],
                    "num_samples": len(stats.time_series.values)
                }
            
            metric_data.append(data)
        
        results_dict[metric_name] = metric_data
    
    with open(output_path, 'w') as f:
        json.dump(results_dict, f, indent=2)
    
    print(f"\n💾 Results saved to: {output_path}")


def main():
    """Main evaluation function."""
    print("🚀 Starting Planner-Expert Metrics Evaluation")
    print("=" * 50)
    
    # Create output directory
    output_dir = Path("/home/chen/nuplan-devkit/metrics_csv/results")
    output_dir.mkdir(exist_ok=True)
    
    # 1. Create mock simulation data
    print("\n1️⃣  Creating mock SimulationHistory...")
    history, scenario = create_mock_simulation_history(
        num_samples=50,   # 5 seconds at 10Hz
        dt=0.1,           # 10Hz sampling
        trajectory_noise=0.1  # 10cm noise level
    )
    
    print(f"   ✅ Created history with {len(history.data)} samples")
    print(f"   ✅ Expert trajectory: {len(scenario.get_expert_ego_trajectory())} states")
    
    # 2. Setup metrics
    print("\n2️⃣  Setting up planner-expert metrics...")
    metrics = setup_planner_expert_metrics()
    print(f"   ✅ Configured {len(metrics)} metrics:")
    for metric in metrics:
        print(f"      • {metric.name}")
    
    # 3. Create metrics engine
    print("\n3️⃣  Creating MetricsEngine...")
    engine = MetricsEngine(
        main_save_path=output_dir,
        metrics=metrics
    )
    
    # 4. Compute metrics
    print("\n4️⃣  Computing metrics...")
    try:
        metric_results = engine.compute_metric_results(history, scenario)
        print(f"   ✅ Successfully computed {len(metric_results)} metrics")
    except Exception as e:
        print(f"   ❌ Error computing metrics: {e}")
        import traceback
        traceback.print_exc()
        return
    
    # 5. Display results
    print_metric_results(metric_results)
    
    # 6. Save results
    json_output = output_dir / "planner_expert_metrics_results.json"
    save_results_to_json(metric_results, json_output)
    
    # 7. Summary
    print("\n" + "="*80)
    print("📈 SUMMARY")
    print("="*80)
    
    total_score = 0
    for metric_name, stats_list in metric_results.items():
        for stats in stats_list:
            if stats.metric_score is not None:
                total_score += stats.metric_score
                print(f"  {metric_name}: {stats.metric_score:.4f}")
    
    avg_score = total_score / len(metric_results)
    print(f"\n  Average Score: {avg_score:.4f}")
    
    if avg_score > 0.8:
        print("  🎉 Excellent planner performance!")
    elif avg_score > 0.6:
        print("  👍 Good planner performance")
    elif avg_score > 0.4:
        print("  ⚠️  Moderate planner performance")
    else:
        print("  ⛔ Poor planner performance - needs improvement")
    
    print(f"\n✅ Evaluation complete! Check results in: {output_dir}")


if __name__ == "__main__":
    main()