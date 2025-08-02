#!/usr/bin/env python3
"""
CSV-Based Evaluation Pipeline

Complete evaluation pipeline that integrates CSV planner data with nuPlan
database scenarios for metrics computation.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import pandas as pd
import numpy as np
import json
from typing import Dict, List, Tuple, Optional, Any
from pathlib import Path
from dataclasses import dataclass

from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_builder import NuPlanScenarioBuilder
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario_utils import ScenarioMapping
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

from src.planning.csv_data_analyzer import CSVDataAnalyzer, CSVColumnMapping
from src.planning.csv_trajectory import CSVEgoStateConverter, CSVTrajectory
from src.planning.scenario_matcher import ScenarioMatcher, ScenarioMatch, MatchingConfig


@dataclass
class EvaluationConfig:
    """Configuration for CSV-based evaluation pipeline."""
    
    # Database configuration
    data_root: str = "/home/chen/nuplan-devkit/data/cache/mini"
    map_root: str = "/home/chen/nuplan-devkit/maps"
    
    # Scenario filtering
    scenario_types: Optional[List[str]] = None  # None = all types
    scenario_limit: int = 10  # Maximum scenarios to evaluate
    min_duration_s: float = 2.0  # Minimum scenario duration
    
    # Matching configuration
    matching_config: Optional[MatchingConfig] = None
    
    # Metrics configuration
    comparison_horizon: List[int] = None  # Will default to [1] for stability
    comparison_frequency: int = 1  # Hz
    
    # Evaluation thresholds
    max_average_l2_error_threshold: float = 2.0
    max_average_heading_error_threshold: float = 0.3
    max_final_l2_error_threshold: float = 3.0
    max_final_heading_error_threshold: float = 0.5
    max_displacement_threshold: List[float] = None  # Will match horizon length
    max_miss_rate_threshold: float = 0.2
    
    # Output configuration
    results_dir: str = "csv_evaluation_results"
    save_detailed_results: bool = True
    save_trajectory_data: bool = False


class CSVEvaluationPipeline:
    """
    Complete evaluation pipeline for CSV planner data against nuPlan expert trajectories.
    
    This pipeline:
    1. Analyzes CSV data structure
    2. Loads nuPlan database scenarios
    3. Matches CSV data to appropriate scenarios
    4. Converts CSV data to nuPlan trajectory format
    5. Computes planner-expert comparison metrics
    6. Generates comprehensive evaluation reports
    """
    
    def __init__(
        self,
        csv_file_path: str,
        config: Optional[EvaluationConfig] = None
    ):
        """
        Initialize evaluation pipeline.
        
        Args:
            csv_file_path: Path to CSV file containing planner data
            config: Evaluation configuration (uses defaults if None)
        """
        self.csv_file_path = Path(csv_file_path)
        self.config = config or EvaluationConfig()
        
        # Set default values for config
        if self.config.comparison_horizon is None:
            self.config.comparison_horizon = [1]
        if self.config.max_displacement_threshold is None:
            self.config.max_displacement_threshold = [1.0] * len(self.config.comparison_horizon)
        if self.config.matching_config is None:
            self.config.matching_config = MatchingConfig()
        
        # Initialize components
        self.csv_data: Optional[pd.DataFrame] = None
        self.column_mapping: Optional[CSVColumnMapping] = None
        self.scenario_builder: Optional[NuPlanScenarioBuilder] = None
        self.available_scenarios: List = []
        self.scenario_matches: List[ScenarioMatch] = []
        self.evaluation_results: List[Dict[str, Any]] = []
        
        # Create results directory
        self.results_dir = Path(self.config.results_dir)
        self.results_dir.mkdir(exist_ok=True)
    
    def run_complete_evaluation(self) -> Dict[str, Any]:
        """
        Run the complete evaluation pipeline.
        
        Returns:
            Dictionary containing evaluation summary and results
        """
        print("🚀 Starting CSV-based Evaluation Pipeline")
        print("=" * 60)
        
        try:
            # Step 1: Analyze CSV data
            self._analyze_csv_data()
            
            # Step 2: Initialize database connection
            self._initialize_database()
            
            # Step 3: Load and filter scenarios
            self._load_scenarios()
            
            # Step 4: Match CSV data to scenarios
            self._match_scenarios()
            
            # Step 5: Evaluate matched scenarios
            self._evaluate_scenarios()
            
            # Step 6: Generate summary and save results
            summary = self._generate_summary()
            self._save_results(summary)
            
            print("✅ Evaluation pipeline completed successfully!")
            return summary
            
        except Exception as e:
            print(f"❌ Evaluation pipeline failed: {e}")
            raise
    
    def _analyze_csv_data(self):
        """Analyze CSV data structure and generate column mapping."""
        print("\n1️⃣ Analyzing CSV data structure...")
        
        # Initialize analyzer
        analyzer = CSVDataAnalyzer(str(self.csv_file_path))
        
        # Load CSV
        if not analyzer.load_csv():
            raise RuntimeError(f"Failed to load CSV file: {self.csv_file_path}")
        
        self.csv_data = analyzer.df
        print(f"   ✓ Loaded CSV with {len(self.csv_data)} rows, {len(self.csv_data.columns)} columns")
        
        # Analyze structure
        analyzer.analyze_structure()
        self.column_mapping = analyzer.generate_mapping_recommendations()
        
        # Print mapping summary
        print("   📋 Column Mapping:")
        for field_name, value in self.column_mapping.__dict__.items():
            if value is not None:
                print(f"      {field_name}: {value}")
        
        # Export analysis
        analysis_path = self.results_dir / "csv_analysis.json"
        analyzer.export_analysis(str(analysis_path))
        print(f"   💾 Analysis saved to: {analysis_path}")
    
    def _initialize_database(self):
        """Initialize nuPlan database connection."""
        print("\n2️⃣ Initializing nuPlan database connection...")
        
        try:
            # Create scenario mapping
            scenario_mapping = ScenarioMapping(
                scenario_map=None,  # Use default
                subsample_ratio_override=None
            )
            
            # Initialize scenario builder
            self.scenario_builder = NuPlanScenarioBuilder(
                data_root=self.config.data_root,
                map_root=self.config.map_root,
                db_files=None,  # Auto-discover
                map_version="nuplan-maps-v1.0",
                scenario_mapping=scenario_mapping
            )
            
            print(f"   ✓ Connected to database at: {self.config.data_root}")
            
        except Exception as e:
            raise RuntimeError(f"Failed to initialize database: {e}")
    
    def _load_scenarios(self):
        """Load and filter scenarios from database."""
        print("\n3️⃣ Loading scenarios from database...")
        
        try:
            # Create scenario filter
            scenario_filter = ScenarioFilter(
                scenario_types=self.config.scenario_types,
                scenario_tokens=None,
                log_names=None,
                map_names=None,
                num_scenarios_per_type=self.config.scenario_limit,
                limit_total_scenarios=self.config.scenario_limit * 5 if self.config.scenario_types else self.config.scenario_limit,
                expand_scenarios=False,
                filter_invalid_goals=True,
                shuffle=False,
                timestamp_threshold_s=self.config.min_duration_s
            )
            
            # Get scenarios
            worker = Sequential()
            self.available_scenarios = self.scenario_builder.get_scenarios(scenario_filter, worker)
            
            print(f"   ✓ Loaded {len(self.available_scenarios)} scenarios")
            
            # Print scenario summary
            scenario_types = {}
            for scenario in self.available_scenarios:
                scenario_type = scenario.scenario_type
                scenario_types[scenario_type] = scenario_types.get(scenario_type, 0) + 1
            
            print("   📊 Scenario types:")
            for scenario_type, count in scenario_types.items():
                print(f"      {scenario_type}: {count}")
                
        except Exception as e:
            raise RuntimeError(f"Failed to load scenarios: {e}")
    
    def _match_scenarios(self):
        """Match CSV data to database scenarios."""
        print("\n4️⃣ Matching CSV data to database scenarios...")
        
        # Initialize matcher
        matcher = ScenarioMatcher(
            csv_data=self.csv_data,
            column_mapping=self.column_mapping,
            config=self.config.matching_config
        )
        
        # Find matches
        self.scenario_matches = matcher.find_best_matches(self.available_scenarios)
        
        print(f"   ✓ Found {len(self.scenario_matches)} scenario matches")
        
        # Print match summary
        for match in self.scenario_matches:
            print(f"      {match.scenario_id} → {match.scenario_name}")
            print(f"         Method: {match.matching_method}, Confidence: {match.matching_confidence:.2f}")
            print(f"         CSV rows: {len(match.csv_rows)}, Expert states: {match.expert_trajectory_length}")
        
        # Export matches
        matches_path = self.results_dir / "scenario_matches.json"
        matcher.matches = self.scenario_matches  # Set matches for export
        matcher.export_matches(str(matches_path))
        print(f"   💾 Matches saved to: {matches_path}")
        
        if not self.scenario_matches:
            raise RuntimeError("No scenario matches found - cannot proceed with evaluation")
    
    def _evaluate_scenarios(self):
        """Evaluate matched scenarios using metrics engine."""
        print("\n5️⃣ Evaluating matched scenarios...")
        
        # Initialize converter
        converter = CSVEgoStateConverter(self.column_mapping)
        
        # Setup metrics
        metrics = self._setup_metrics()
        
        for i, match in enumerate(self.scenario_matches):
            print(f"\n--- Evaluating scenario {i+1}/{len(self.scenario_matches)} ---")
            print(f"   Scenario: {match.scenario_name}")
            print(f"   CSV data: {len(match.csv_rows)} rows")
            
            try:
                # Get database scenario
                db_scenario = next(s for s in self.available_scenarios if s.scenario_name == match.scenario_name)
                
                # Create simulation history from CSV data
                history = self._create_simulation_history_from_csv(
                    match, db_scenario, converter
                )
                
                # Compute metrics
                print(f"   🧮 Computing metrics...")
                
                metrics_engine = MetricsEngine(metrics=metrics, main_save_path=None, timestamp=None)
                computed_metrics = metrics_engine.compute(history_list=[history])
                
                # Process results
                scenario_result = {
                    "scenario_id": match.scenario_id,
                    "scenario_name": match.scenario_name,
                    "scenario_type": match.metadata.get("scenario_type", "unknown"),
                    "matching_method": match.matching_method,
                    "matching_confidence": match.matching_confidence,
                    "csv_rows_count": len(match.csv_rows),
                    "expert_states_count": match.expert_trajectory_length,
                    "metrics": {}
                }
                
                # Extract metric scores
                for metric_name, metric_result in computed_metrics.items():
                    if hasattr(metric_result, 'get_score') and callable(metric_result.get_score):
                        score = metric_result.get_score()
                        scenario_result["metrics"][metric_name] = {
                            "score": float(score),
                            "category": "accuracy"
                        }
                        print(f"      {metric_name}: {score:.4f}")
                
                self.evaluation_results.append(scenario_result)
                print(f"   ✅ Scenario evaluation completed")
                
            except Exception as e:
                print(f"   ❌ Failed to evaluate scenario {match.scenario_name}: {e}")
                # Add failed result
                failed_result = {
                    "scenario_id": match.scenario_id,
                    "scenario_name": match.scenario_name,
                    "scenario_type": match.metadata.get("scenario_type", "unknown"),
                    "matching_method": match.matching_method,
                    "matching_confidence": match.matching_confidence,
                    "csv_rows_count": len(match.csv_rows),
                    "expert_states_count": match.expert_trajectory_length,
                    "error": str(e),
                    "metrics": {}
                }
                self.evaluation_results.append(failed_result)
    
    def _create_simulation_history_from_csv(
        self,
        match: ScenarioMatch,
        db_scenario,
        converter: CSVEgoStateConverter
    ) -> SimulationHistory:
        """
        Create simulation history from CSV data and database scenario.
        
        Args:
            match: Scenario match information
            db_scenario: Database scenario object
            converter: CSV to EgoState converter
            
        Returns:
            SimulationHistory object
        """
        # Get CSV data for this match
        csv_subset = self.csv_data.iloc[match.csv_rows]
        
        # Get expert trajectory
        expert_trajectory = list(db_scenario.get_expert_ego_trajectory())
        
        # Create simulation history
        history = SimulationHistory(db_scenario.map_api, db_scenario.get_mission_goal())
        
        # Determine alignment strategy
        csv_length = len(csv_subset)
        expert_length = len(expert_trajectory)
        
        # Use the shorter sequence length to avoid indexing errors
        max_iterations = min(csv_length, expert_length)
        
        print(f"      Creating history with {max_iterations} iterations")
        print(f"      CSV length: {csv_length}, Expert length: {expert_length}")
        
        for i in range(max_iterations):
            try:
                # Get CSV row and expert state
                csv_row = csv_subset.iloc[i]
                expert_state = expert_trajectory[i]
                
                # Convert CSV row to planner ego state
                planner_ego_state = converter.convert_row_to_ego_state(
                    csv_row,
                    base_timestamp=expert_state.time_point.time_s + match.time_alignment_offset
                )
                
                # Create trajectory for this timestep
                if self.column_mapping.horizon_prefix:
                    try:
                        trajectory = converter.convert_horizon_to_trajectory(
                            csv_row,
                            base_timestamp=planner_ego_state.time_point.time_s,
                            dt=0.1
                        )
                    except:
                        # Fallback: create single-point trajectory
                        from .csv_trajectory import CSVTrajectoryPoint, CSVTrajectory
                        single_point = CSVTrajectoryPoint(
                            timestamp=planner_ego_state.time_point.time_s,
                            x=planner_ego_state.rear_axle.x,
                            y=planner_ego_state.rear_axle.y,
                            heading=planner_ego_state.rear_axle.heading
                        )
                        trajectory = CSVTrajectory([single_point])
                else:
                    # Create single-point trajectory
                    from .csv_trajectory import CSVTrajectoryPoint, CSVTrajectory
                    single_point = CSVTrajectoryPoint(
                        timestamp=planner_ego_state.time_point.time_s,
                        x=planner_ego_state.rear_axle.x,
                        y=planner_ego_state.rear_axle.y,
                        heading=planner_ego_state.rear_axle.heading
                    )
                    trajectory = CSVTrajectory([single_point])
                
                # Create simulation iteration
                iteration = SimulationIteration(
                    time_point=expert_state.time_point,
                    index=i
                )
                
                # Get observations
                observation = db_scenario.get_tracked_objects_at_iteration(i)
                traffic_lights = db_scenario.get_traffic_light_status_at_iteration(i)
                
                # Create simulation sample
                sample = SimulationHistorySample(
                    iteration=iteration,
                    ego_state=planner_ego_state,  # Use planner state as ego
                    trajectory=trajectory,
                    observation=observation,
                    traffic_light_status=traffic_lights
                )
                
                history.add_sample(sample)
                
            except Exception as e:
                print(f"         Warning: Failed to process iteration {i}: {e}")
                continue
        
        print(f"      ✓ Created simulation history with {len(history.data)} samples")
        return history
    
    def _setup_metrics(self) -> List:
        """Setup metrics for evaluation."""
        print("   📊 Setting up metrics...")
        
        # Configuration
        comparison_horizon = self.config.comparison_horizon
        comparison_frequency = self.config.comparison_frequency
        max_displacement_threshold = self.config.max_displacement_threshold
        
        # Create metrics
        metrics = [
            PlannerExpertAverageL2ErrorStatistics(
                name="planner_expert_average_l2_error",
                category="accuracy",
                comparison_horizon=comparison_horizon,
                comparison_frequency=comparison_frequency,
                max_average_l2_error_threshold=self.config.max_average_l2_error_threshold
            ),
            PlannerExpertAverageHeadingErrorStatistics(
                name="planner_expert_average_heading_error",
                category="accuracy",
                comparison_horizon=comparison_horizon,
                comparison_frequency=comparison_frequency,
                max_average_heading_error_threshold=self.config.max_average_heading_error_threshold
            ),
            PlannerExpertFinalL2ErrorStatistics(
                name="planner_expert_final_l2_error",
                category="accuracy",
                comparison_horizon=comparison_horizon,
                comparison_frequency=comparison_frequency,
                max_final_l2_error_threshold=self.config.max_final_l2_error_threshold
            ),
            PlannerExpertFinalHeadingErrorStatistics(
                name="planner_expert_final_heading_error",
                category="accuracy",
                comparison_horizon=comparison_horizon,
                comparison_frequency=comparison_frequency,
                max_final_heading_error_threshold=self.config.max_final_heading_error_threshold
            ),
            PlannerMissRateStatistics(
                name="planner_miss_rate",
                category="accuracy",
                comparison_horizon=comparison_horizon,
                comparison_frequency=comparison_frequency,
                max_displacement_threshold=max_displacement_threshold,
                max_miss_rate_threshold=self.config.max_miss_rate_threshold
            )
        ]
        
        print(f"   ✓ Configured {len(metrics)} metrics")
        return metrics
    
    def _generate_summary(self) -> Dict[str, Any]:
        """Generate evaluation summary."""
        print("\n6️⃣ Generating evaluation summary...")
        
        if not self.evaluation_results:
            return {"error": "No evaluation results available"}
        
        # Calculate aggregate metrics
        successful_results = [r for r in self.evaluation_results if "error" not in r]
        
        if not successful_results:
            return {"error": "No successful evaluations"}
        
        # Aggregate scores
        aggregate_metrics = {}
        metric_names = set()
        
        for result in successful_results:
            for metric_name in result["metrics"].keys():
                metric_names.add(metric_name)
        
        for metric_name in metric_names:
            scores = [r["metrics"][metric_name]["score"] for r in successful_results 
                     if metric_name in r["metrics"]]
            if scores:
                aggregate_metrics[metric_name] = {
                    "mean": float(np.mean(scores)),
                    "std": float(np.std(scores)),
                    "min": float(np.min(scores)),
                    "max": float(np.max(scores)),
                    "count": len(scores)
                }
        
        # Overall performance
        if aggregate_metrics:
            overall_scores = [metrics["mean"] for metrics in aggregate_metrics.values()]
            overall_performance = float(np.mean(overall_scores))
        else:
            overall_performance = 0.0
        
        # Determine performance level
        if overall_performance >= 0.95:
            performance_level = "Excellent"
        elif overall_performance >= 0.85:
            performance_level = "Good"
        elif overall_performance >= 0.70:
            performance_level = "Fair"
        else:
            performance_level = "Poor"
        
        summary = {
            "evaluation_timestamp": pd.Timestamp.now().isoformat(),
            "csv_file": str(self.csv_file_path),
            "total_scenarios_attempted": len(self.evaluation_results),
            "successful_evaluations": len(successful_results),
            "failed_evaluations": len(self.evaluation_results) - len(successful_results),
            "overall_performance": overall_performance,
            "performance_level": performance_level,
            "aggregate_metrics": aggregate_metrics,
            "scenario_results": self.evaluation_results,
            "configuration": {
                "data_root": self.config.data_root,
                "scenario_limit": self.config.scenario_limit,
                "comparison_horizon": self.config.comparison_horizon,
                "matching_config": self.config.matching_config.__dict__
            }
        }
        
        print(f"   ✓ Summary generated for {len(successful_results)} successful evaluations")
        print(f"   📊 Overall performance: {overall_performance:.1%} ({performance_level})")
        
        return summary
    
    def _save_results(self, summary: Dict[str, Any]):
        """Save evaluation results to files."""
        print("\n💾 Saving evaluation results...")
        
        # Save main results
        results_path = self.results_dir / "evaluation_results.json"
        with open(results_path, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"   ✓ Main results saved to: {results_path}")
        
        # Save detailed results if enabled
        if self.config.save_detailed_results:
            detailed_path = self.results_dir / "detailed_results.json"
            detailed_results = {
                "summary": summary,
                "scenario_matches": [
                    {
                        "scenario_id": m.scenario_id,
                        "scenario_name": m.scenario_name,
                        "csv_rows": m.csv_rows,
                        "expert_trajectory_length": m.expert_trajectory_length,
                        "time_alignment_offset": m.time_alignment_offset,
                        "matching_confidence": m.matching_confidence,
                        "matching_method": m.matching_method,
                        "metadata": m.metadata
                    }
                    for m in self.scenario_matches
                ],
                "column_mapping": self.column_mapping.__dict__ if self.column_mapping else None
            }
            
            with open(detailed_path, 'w') as f:
                json.dump(detailed_results, f, indent=2)
            print(f"   ✓ Detailed results saved to: {detailed_path}")
        
        print(f"   📁 All results saved to directory: {self.results_dir}")


def main():
    """Main function for testing the evaluation pipeline."""
    print("CSV Evaluation Pipeline - Test Mode")
    print("This module provides complete CSV planner data evaluation against nuPlan expert trajectories.")


if __name__ == "__main__":
    main()