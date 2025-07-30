#!/usr/bin/env python3
"""
Offline Trajectory Evaluation Runner

This script runs offline evaluation of trajectory data using nuPlan's open-loop simulation infrastructure.
It leverages the existing simulation pipeline with a CSV-based LogFuturePlanner.
"""

import os
import sys
import logging
import argparse
from pathlib import Path
from typing import List, Optional

# Add nuPlan and offline_evaluation to Python path
nuplan_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(nuplan_root))
sys.path.insert(0, str(nuplan_root / "offline_evaluation"))

from hydra import compose, initialize_config_dir
from hydra.core.global_hydra import GlobalHydra
from omegaconf import DictConfig, OmegaConf

from nuplan.planning.script.run_simulation import main as run_simulation_main

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class OfflineEvaluationRunner:
    """Runner for offline trajectory evaluation using nuPlan's simulation pipeline."""
    
    def __init__(self, csv_file_path: str, output_dir: str = "offline_results"):
        """
        Initialize the offline evaluation runner.
        
        :param csv_file_path: Path to CSV trajectory file
        :param output_dir: Output directory for results
        """
        self.csv_file_path = Path(csv_file_path)
        self.output_dir = Path(output_dir)
        
        # Validate inputs
        if not self.csv_file_path.exists():
            raise FileNotFoundError(f"CSV file not found: {self.csv_file_path}")
        
        self.output_dir.mkdir(parents=True, exist_ok=True)
        logger.info(f"Offline evaluation output directory: {self.output_dir}")
    
    def run_evaluation(
        self, 
        experiment_name: str = "offline_trajectory_evaluation",
        limit_scenarios: Optional[int] = None
    ) -> bool:
        """
        Run offline evaluation using nuPlan's simulation pipeline.
        
        :param experiment_name: Name for the evaluation experiment
        :param limit_scenarios: Limit number of scenarios to evaluate
        :return: True if successful
        """
        try:
            # Clear any existing Hydra instance
            GlobalHydra.instance().clear()
            
            # Get configuration paths
            config_dir = nuplan_root / "nuplan/planning/script/config"
            offline_config_dir = nuplan_root / "offline_evaluation/config"
            
            # Initialize Hydra with nuPlan's config directory
            with initialize_config_dir(config_dir=str(config_dir)):
                
                # Create configuration overrides
                overrides = [
                    # Use existing open_loop_boxes experiment as base
                    f"experiment=open_loop_boxes",
                    
                    # Override planner to use our CSV planner
                    f"planner=../../../offline_evaluation/config/csv_log_future_planner",
                    
                    # Set CSV file path in planner config
                    f"planner.csv_file_path={self.csv_file_path.absolute()}",
                    
                    # Set output directory
                    f"output_dir={self.output_dir.absolute()}",
                    f"experiment_name={experiment_name}",
                    
                    # Use minimal scenarios from database (CSV provides trajectory data)
                    f"scenario_builder=nuplan_mini",
                    f"scenario_filter=one_of_each_scenario_type",
                ]
                
                # Add scenario limit if specified
                if limit_scenarios is not None:
                    overrides.append(f"scenario_builder.scenario_filter.limit_total_scenarios={limit_scenarios}")
                
                # Compose configuration
                cfg = compose(config_name="simulation/default_simulation", overrides=overrides)
                
                logger.info("Starting offline evaluation with nuPlan simulation pipeline...")
                logger.info(f"CSV file: {self.csv_file_path}")
                logger.info(f"Output directory: {self.output_dir}")
                logger.info(f"Configuration overrides: {overrides}")
                
                # Run simulation using nuPlan's standard pipeline
                run_simulation_main(cfg)
                
                logger.info("Offline evaluation completed successfully!")
                return True
                
        except Exception as e:
            logger.error(f"Offline evaluation failed: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # Clean up Hydra
            GlobalHydra.instance().clear()
    
    def run_simple_evaluation(
        self,
        scenarios: Optional[List[str]] = None,
        experiment_name: str = "simple_offline_evaluation"
    ) -> bool:
        """
        Run a simplified offline evaluation without full nuPlan simulation.
        
        :param scenarios: List of scenario IDs to evaluate
        :param experiment_name: Name for the evaluation experiment
        :return: True if successful
        """
        try:
            from planners.csv_log_future_planner import CSVLogFuturePlanner
            
            # Create planner
            planner = CSVLogFuturePlanner(
                csv_file_path=str(self.csv_file_path),
                num_poses=16,
                future_time_horizon=8.0
            )
            
            logger.info(f"Created CSV planner with {len(planner.get_available_scenarios())} scenarios")
            
            # If no specific scenarios requested, use all available
            if scenarios is None:
                scenarios = planner.get_available_scenarios()[:5]  # Limit to first 5 for testing
            
            # Simple evaluation loop
            results = []
            for scenario_id in scenarios:
                logger.info(f"Evaluating scenario: {scenario_id}")
                
                try:
                    planner.set_current_scenario(scenario_id)
                    
                    # Create dummy planner input for testing
                    from nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
                    from nuplan.common.actor_state.state_representation import TimePoint
                    from nuplan.planning.simulation.planner.abstract_planner import PlannerInput
                    
                    iteration = SimulationIteration(index=0, time_point=TimePoint(0))
                    planner_input = PlannerInput(
                        iteration=iteration,
                        history=None,  # Not used by CSV planner
                        traffic_light_data=None
                    )
                    
                    # Generate trajectory
                    trajectory = planner.compute_planner_trajectory(planner_input)
                    
                    # Basic validation
                    sampled_trajectory = trajectory.get_sampled_trajectory()
                    duration = trajectory.duration
                    
                    result = {
                        'scenario_id': scenario_id,
                        'success': True,
                        'trajectory_points': len(sampled_trajectory),
                        'duration_s': duration,
                        'start_time': trajectory.start_time.time_s,
                        'end_time': trajectory.end_time.time_s
                    }
                    
                    results.append(result)
                    logger.info(f"  ✓ Success: {len(sampled_trajectory)} points, {duration:.1f}s")
                    
                except Exception as e:
                    logger.error(f"  ✗ Failed: {e}")
                    results.append({
                        'scenario_id': scenario_id,
                        'success': False,
                        'error': str(e)
                    })
            
            # Save results
            self._save_simple_results(results, experiment_name)
            self._print_simple_summary(results)
            
            return True
            
        except Exception as e:
            logger.error(f"Simple evaluation failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _save_simple_results(self, results: List[dict], experiment_name: str) -> None:
        """Save simple evaluation results."""
        import pandas as pd
        import json
        
        results_dir = self.output_dir / experiment_name
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # Save as CSV
        df = pd.DataFrame(results)
        csv_file = results_dir / "evaluation_results.csv"
        df.to_csv(csv_file, index=False)
        
        # Save as JSON
        json_file = results_dir / "evaluation_results.json"
        with open(json_file, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        logger.info(f"Saved results to: {results_dir}")
    
    def _print_simple_summary(self, results: List[dict]) -> None:
        """Print simple evaluation summary."""
        total = len(results)
        successful = [r for r in results if r.get('success', False)]
        failed = [r for r in results if not r.get('success', False)]
        
        logger.info("\n" + "="*60)
        logger.info("OFFLINE EVALUATION SUMMARY")
        logger.info("="*60)
        logger.info(f"Total scenarios: {total}")
        logger.info(f"Successful: {len(successful)} ({len(successful)/total*100:.1f}%)")
        logger.info(f"Failed: {len(failed)} ({len(failed)/total*100:.1f}%)")
        
        if successful:
            durations = [r['duration_s'] for r in successful if 'duration_s' in r]
            points = [r['trajectory_points'] for r in successful if 'trajectory_points' in r]
            
            if durations and points:
                logger.info(f"\nTrajectory Statistics:")
                logger.info(f"  Average duration: {sum(durations)/len(durations):.1f}s")
                logger.info(f"  Average points: {sum(points)/len(points):.1f}")
        
        logger.info("="*60)


def main():
    """Main function to run offline evaluation."""
    parser = argparse.ArgumentParser(description="Run offline trajectory evaluation")
    parser.add_argument(
        "--csv-file", "-c",
        required=True,
        help="Path to CSV trajectory file"
    )
    parser.add_argument(
        "--output-dir", "-o",
        default="offline_results",
        help="Output directory for results"
    )
    parser.add_argument(
        "--experiment-name", "-e",
        default="offline_trajectory_evaluation",
        help="Name for the evaluation experiment"
    )
    parser.add_argument(
        "--limit-scenarios", "-l",
        type=int,
        help="Limit number of scenarios to evaluate"
    )
    parser.add_argument(
        "--simple-mode", "-s",
        action="store_true",
        help="Run simplified evaluation without full nuPlan simulation"
    )
    parser.add_argument(
        "--scenarios",
        nargs="*",
        help="Specific scenario IDs to evaluate (for simple mode)"
    )
    
    args = parser.parse_args()
    
    # Validate CSV file exists
    if not Path(args.csv_file).exists():
        logger.error(f"CSV file not found: {args.csv_file}")
        return 1
    
    try:
        runner = OfflineEvaluationRunner(args.csv_file, args.output_dir)
        
        if args.simple_mode:
            success = runner.run_simple_evaluation(
                scenarios=args.scenarios,
                experiment_name=args.experiment_name
            )
        else:
            success = runner.run_evaluation(
                experiment_name=args.experiment_name,
                limit_scenarios=args.limit_scenarios
            )
        
        if success:
            print(f"\n✅ Offline evaluation completed successfully!")
            print(f"📊 Results available in: {runner.output_dir}")
        else:
            print(f"\n❌ Offline evaluation failed!")
            return 1
            
    except Exception as e:
        logger.error(f"Evaluation failed: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())