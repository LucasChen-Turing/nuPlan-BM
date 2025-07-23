#!/usr/bin/env python3
"""
nuPlan Planner Benchmark Script

This script evaluates multiple planners on nuPlan scenarios and generates comparative results.
It tests SimplePlanner, IDMPlanner, and can be extended for ML planners.
"""

import os
import sys
import logging
import tempfile
from pathlib import Path
from typing import Dict, List, Any
import yaml
import pandas as pd
from datetime import datetime

# Add nuplan to path
sys.path.append('/home/chen/nuplan-devkit')

import hydra
from omegaconf import DictConfig, OmegaConf
from nuplan.planning.script.run_simulation import run_simulation
from nuplan.planning.simulation.planner.simple_planner import SimplePlanner
from nuplan.planning.simulation.planner.idm_planner import IDMPlanner

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PlannerBenchmark:
    """Class to handle planner benchmarking operations."""
    
    def __init__(self, base_config_path: str, output_dir: str):
        """
        Initialize the benchmark.
        :param base_config_path: Path to base configuration file
        :param output_dir: Directory to store benchmark results
        """
        self.base_config_path = base_config_path
        self.output_dir = Path(output_dir)
        self.results_dir = self.output_dir / "results"
        self.results_dir.mkdir(parents=True, exist_ok=True)
        
        # Define planners to benchmark
        self.planners_config = {
            'SimplePlanner': {
                'name': 'simple_planner',
                'config': {
                    '_target_': 'nuplan.planning.simulation.planner.simple_planner.SimplePlanner',
                    'horizon_seconds': 8.0,
                    'sampling_time': 0.25,
                    'acceleration': [0.0, 0.0],
                    'max_velocity': 5.0,
                    'steering_angle': 0.0
                }
            },
            'IDMPlanner': {
                'name': 'idm_planner',
                'config': {
                    '_target_': 'nuplan.planning.simulation.planner.idm_planner.IDMPlanner',
                    'target_velocity': 10.0,
                    'min_gap_to_lead_agent': 1.0,
                    'headway_time': 1.5,
                    'accel_max': 1.0,
                    'decel_max': 3.0,
                    'planned_trajectory_samples': 16,
                    'planned_trajectory_sample_interval': 0.5,
                    'occupancy_map_radius': 40.0
                }
            }
        }
        
        self.benchmark_results = {}
    
    def load_base_config(self) -> DictConfig:
        """Load the base configuration file."""
        with open(self.base_config_path, 'r') as f:
            config_dict = yaml.safe_load(f)
        return OmegaConf.create(config_dict)
    
    def create_planner_config(self, planner_name: str, base_config: DictConfig) -> DictConfig:
        """
        Create a configuration for a specific planner.
        :param planner_name: Name of the planner
        :param base_config: Base configuration
        :return: Modified configuration for the planner
        """
        config = base_config.copy()
        
        # Set planner-specific configuration
        planner_cfg = self.planners_config[planner_name]
        config.planner = OmegaConf.create({planner_cfg['name']: planner_cfg['config']})
        
        # Set output directory for this planner
        planner_output_dir = str(self.results_dir / planner_name.lower())
        config.output_dir = planner_output_dir
        
        # Set experiment name
        config.experiment_name = f'benchmark_{planner_name.lower()}'
        
        return config
    
    def run_planner_simulation(self, planner_name: str) -> bool:
        """
        Run simulation for a specific planner.
        :param planner_name: Name of the planner to run
        :return: True if successful, False otherwise
        """
        logger.info(f"Running benchmark for {planner_name}...")
        
        try:
            # Load base config and modify for this planner
            base_config = self.load_base_config()
            planner_config = self.create_planner_config(planner_name, base_config)
            
            # Create temporary config file
            with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as tmp_file:
                yaml.dump(OmegaConf.to_yaml(planner_config), tmp_file)
                tmp_config_path = tmp_file.name
            
            # Run simulation using hydra
            os.environ['PYTHONPATH'] = '/home/chen/nuplan-devkit'
            
            @hydra.main(version_base=None, config_path=None)
            def run_with_hydra(cfg: DictConfig) -> None:
                run_simulation(cfg)
            
            # Initialize Hydra and run
            with hydra.initialize_config_dir(config_dir=str(Path(tmp_config_path).parent)):
                cfg = OmegaConf.load(tmp_config_path)
                run_simulation(cfg)
            
            # Clean up temporary file
            os.unlink(tmp_config_path)
            
            logger.info(f"Successfully completed benchmark for {planner_name}")
            return True
            
        except Exception as e:
            logger.error(f"Error running benchmark for {planner_name}: {str(e)}")
            return False
    
    def collect_results(self) -> Dict[str, Any]:
        """
        Collect and analyze results from all planner runs.
        :return: Dictionary containing benchmark results
        """
        results = {}
        
        for planner_name in self.planners_config.keys():
            planner_dir = self.results_dir / planner_name.lower()
            
            # Look for metric files
            metric_files = list(planner_dir.glob("**/metrics/*.parquet"))
            
            if metric_files:
                logger.info(f"Found {len(metric_files)} metric files for {planner_name}")
                
                # Load and aggregate metrics
                all_metrics = []
                for metric_file in metric_files:
                    try:
                        df = pd.read_parquet(metric_file)
                        df['planner'] = planner_name
                        all_metrics.append(df)
                    except Exception as e:
                        logger.warning(f"Error loading {metric_file}: {e}")
                
                if all_metrics:
                    combined_metrics = pd.concat(all_metrics, ignore_index=True)
                    results[planner_name] = combined_metrics
                    
                    # Save combined metrics
                    output_file = self.results_dir / f"{planner_name}_metrics.parquet"
                    combined_metrics.to_parquet(output_file)
                    logger.info(f"Saved combined metrics for {planner_name} to {output_file}")
            else:
                logger.warning(f"No metric files found for {planner_name}")
        
        return results
    
    def generate_comparison_report(self, results: Dict[str, Any]) -> None:
        """
        Generate a comparison report of all planners.
        :param results: Dictionary of results from all planners
        """
        if not results:
            logger.warning("No results to compare")
            return
        
        # Combine all results
        all_results = []
        for planner_name, df in results.items():
            df_copy = df.copy()
            df_copy['planner'] = planner_name
            all_results.append(df_copy)
        
        combined_df = pd.concat(all_results, ignore_index=True)
        
        # Generate summary statistics
        summary_stats = {}
        
        # Key metrics to analyze
        key_metrics = [
            'ego_expert_l2_error_statistics.mean',
            'ego_is_comfortable.value',
            'drivable_area_compliance.value',
            'no_ego_at_fault_collisions.value',
            'speed_limit_compliance.value'
        ]
        
        for metric in key_metrics:
            if metric in combined_df.columns:
                summary_stats[metric] = combined_df.groupby('planner')[metric].agg(['mean', 'std', 'count'])
        
        # Save summary statistics
        summary_file = self.results_dir / "benchmark_summary.csv"
        summary_df = pd.DataFrame(summary_stats).T
        summary_df.to_csv(summary_file)
        
        # Save combined results
        combined_file = self.results_dir / "combined_benchmark_results.parquet"
        combined_df.to_parquet(combined_file)
        
        # Generate text report
        report_file = self.results_dir / "benchmark_report.txt"
        with open(report_file, 'w') as f:
            f.write(f"nuPlan Planner Benchmark Report\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"="*50 + "\n\n")
            
            f.write(f"Planners tested: {list(results.keys())}\n")
            f.write(f"Total scenarios: {len(combined_df['scenario_name'].unique()) if 'scenario_name' in combined_df.columns else 'Unknown'}\n\n")
            
            f.write("Summary Statistics:\n")
            f.write("-"*20 + "\n")
            f.write(str(summary_df))
            
        logger.info(f"Generated benchmark report: {report_file}")
        logger.info(f"Combined results saved to: {combined_file}")
        logger.info(f"Summary statistics saved to: {summary_file}")
    
    def run_full_benchmark(self) -> None:
        """Run the complete benchmark for all planners."""
        logger.info("Starting nuPlan Planner Benchmark...")
        
        # Run simulation for each planner
        successful_planners = []
        for planner_name in self.planners_config.keys():
            if self.run_planner_simulation(planner_name):
                successful_planners.append(planner_name)
        
        if not successful_planners:
            logger.error("No planners ran successfully!")
            return
        
        logger.info(f"Successfully ran {len(successful_planners)} planners: {successful_planners}")
        
        # Collect and analyze results
        logger.info("Collecting and analyzing results...")
        results = self.collect_results()
        
        # Generate comparison report
        self.generate_comparison_report(results)
        
        logger.info("Benchmark completed successfully!")
        logger.info(f"Results available in: {self.results_dir}")


def main():
    """Main function to run the benchmark."""
    base_config_path = "/home/chen/nuplan-devkit/planner_benchmark/benchmark_config.yaml"
    output_dir = "/home/chen/nuplan-devkit/planner_benchmark"
    
    benchmark = PlannerBenchmark(base_config_path, output_dir)
    benchmark.run_full_benchmark()


if __name__ == "__main__":
    main()