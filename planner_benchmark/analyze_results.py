#!/usr/bin/env python3
"""
nuPlan Benchmark Results Analysis Script

This script analyzes and compares the results from different planner benchmarks.
"""

import os
import sys
import logging
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import Dict, List, Optional
import glob

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Set style for plots
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")


class BenchmarkAnalyzer:
    """Class to analyze benchmark results from multiple planners."""
    
    def __init__(self, results_dir: str):
        """
        Initialize the analyzer.
        :param results_dir: Directory containing benchmark results
        """
        self.results_dir = Path(results_dir)
        self.output_dir = self.results_dir / "analysis"
        self.output_dir.mkdir(exist_ok=True)
        
        self.planner_data = {}
        self.combined_data = None
    
    def load_planner_results(self) -> bool:
        """
        Load results from all planner directories.
        :return: True if any results were loaded
        """
        planner_dirs = [d for d in self.results_dir.iterdir() if d.is_dir() and d.name != "analysis"]
        
        if not planner_dirs:
            logger.warning(f"No planner directories found in {self.results_dir}")
            return False
        
        for planner_dir in planner_dirs:
            planner_name = planner_dir.name.replace('_planner', '').title()
            logger.info(f"Loading results for {planner_name}...")
            
            # Look for metric files
            metric_files = list(planner_dir.glob("**/metrics/*.parquet"))
            
            if not metric_files:
                logger.warning(f"No metric files found for {planner_name}")
                continue
            
            # Load and combine all metric files for this planner
            planner_metrics = []
            for metric_file in metric_files:
                try:
                    df = pd.read_parquet(metric_file)
                    df['planner'] = planner_name
                    df['metric_file'] = str(metric_file)
                    planner_metrics.append(df)
                except Exception as e:
                    logger.warning(f"Error loading {metric_file}: {e}")
            
            if planner_metrics:
                combined_df = pd.concat(planner_metrics, ignore_index=True)
                self.planner_data[planner_name] = combined_df
                logger.info(f"Loaded {len(combined_df)} metric records for {planner_name}")
        
        if self.planner_data:
            # Combine all planner data
            self.combined_data = pd.concat(list(self.planner_data.values()), ignore_index=True)
            logger.info(f"Total combined records: {len(self.combined_data)}")
            return True
        
        return False
    
    def get_key_metrics(self) -> List[str]:
        """Get list of key metrics to analyze."""
        if self.combined_data is None:
            return []
        
        # Define important metrics based on nuPlan documentation
        key_metrics = [
            'ego_expert_l2_error_statistics',
            'ego_is_comfortable', 
            'drivable_area_compliance',
            'no_ego_at_fault_collisions',
            'speed_limit_compliance',
            'time_to_collision_within_bound',
            'ego_progress_along_expert_route',
            'ego_mean_speed_statistics',
            'driving_direction_compliance'
        ]
        
        # Filter to only metrics that exist in the data
        available_metrics = []
        for metric in key_metrics:
            # Look for columns that contain the metric name
            matching_cols = [col for col in self.combined_data.columns if metric in col]
            if matching_cols:
                available_metrics.extend(matching_cols[:3])  # Limit to first 3 matches
        
        return available_metrics
    
    def generate_summary_statistics(self) -> pd.DataFrame:
        """Generate summary statistics for all planners."""
        if self.combined_data is None:
            return pd.DataFrame()
        
        key_metrics = self.get_key_metrics()
        
        if not key_metrics:
            logger.warning("No key metrics found in the data")
            return pd.DataFrame()
        
        # Calculate summary statistics for each planner and metric
        summary_data = []
        
        for planner in self.combined_data['planner'].unique():
            planner_data = self.combined_data[self.combined_data['planner'] == planner]
            
            for metric in key_metrics:
                if metric in planner_data.columns:
                    values = pd.to_numeric(planner_data[metric], errors='coerce').dropna()
                    
                    if len(values) > 0:
                        summary_data.append({
                            'planner': planner,
                            'metric': metric,
                            'count': len(values),
                            'mean': values.mean(),
                            'std': values.std(),
                            'min': values.min(),
                            'max': values.max(),
                            'median': values.median()
                        })
        
        summary_df = pd.DataFrame(summary_data)
        
        # Save summary statistics
        summary_file = self.output_dir / "summary_statistics.csv"
        summary_df.to_csv(summary_file, index=False)
        logger.info(f"Saved summary statistics to {summary_file}")
        
        return summary_df
    
    def create_comparison_plots(self, summary_df: pd.DataFrame) -> None:
        """Create comparison plots for different metrics."""
        if summary_df.empty:
            logger.warning("No summary data available for plotting")
            return
        
        # Get unique metrics
        metrics = summary_df['metric'].unique()
        
        # Create subplots for each metric
        n_metrics = len(metrics)
        if n_metrics == 0:
            return
        
        # Calculate subplot layout
        n_cols = min(3, n_metrics)
        n_rows = (n_metrics + n_cols - 1) // n_cols
        
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(5*n_cols, 4*n_rows))
        if n_metrics == 1:
            axes = [axes]
        elif n_rows == 1:
            axes = [axes]
        else:
            axes = axes.flatten()
        
        for i, metric in enumerate(metrics):
            ax = axes[i] if i < len(axes) else None
            if ax is None:
                break
                
            metric_data = summary_df[summary_df['metric'] == metric]
            
            # Create bar plot
            planners = metric_data['planner']
            means = metric_data['mean']
            stds = metric_data['std']
            
            bars = ax.bar(planners, means, yerr=stds, capsize=5, alpha=0.7)
            ax.set_title(metric.replace('_', ' ').title())
            ax.set_ylabel('Value')
            ax.tick_params(axis='x', rotation=45)
            
            # Add value labels on bars
            for bar, mean_val in zip(bars, means):
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height,
                       f'{mean_val:.3f}', ha='center', va='bottom')
        
        # Hide unused subplots
        for i in range(len(metrics), len(axes)):
            axes[i].set_visible(False)
        
        plt.tight_layout()
        plot_file = self.output_dir / "metric_comparison.png"
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        logger.info(f"Saved comparison plot to {plot_file}")
    
    def generate_detailed_report(self, summary_df: pd.DataFrame) -> None:
        """Generate a detailed text report."""
        report_file = self.output_dir / "detailed_report.txt"
        
        with open(report_file, 'w') as f:
            f.write("nuPlan Planner Benchmark - Detailed Analysis Report\n")
            f.write("=" * 60 + "\n\n")
            
            # Overview
            f.write("OVERVIEW\n")
            f.write("-" * 20 + "\n")
            f.write(f"Total planners evaluated: {len(self.planner_data)}\n")
            f.write(f"Planners: {', '.join(self.planner_data.keys())}\n")
            
            if self.combined_data is not None:
                total_scenarios = self.combined_data.get('scenario_name', pd.Series()).nunique()
                f.write(f"Total scenarios: {total_scenarios}\n")
            
            f.write(f"Total metric records: {len(self.combined_data) if self.combined_data is not None else 0}\n\n")
            
            # Per-planner statistics
            f.write("PER-PLANNER STATISTICS\n")
            f.write("-" * 30 + "\n")
            
            for planner_name, data in self.planner_data.items():
                f.write(f"\n{planner_name}:\n")
                f.write(f"  Records: {len(data)}\n")
                
                # Get numeric columns for basic stats
                numeric_cols = data.select_dtypes(include=[np.number]).columns
                if len(numeric_cols) > 0:
                    f.write(f"  Numeric metrics: {len(numeric_cols)}\n")
            
            # Summary statistics table
            if not summary_df.empty:
                f.write("\n\nSUMMARY STATISTICS\n")
                f.write("-" * 25 + "\n")
                
                # Group by metric for easier reading
                for metric in summary_df['metric'].unique():
                    f.write(f"\n{metric}:\n")
                    metric_data = summary_df[summary_df['metric'] == metric]
                    
                    for _, row in metric_data.iterrows():
                        f.write(f"  {row['planner']}: mean={row['mean']:.4f}, std={row['std']:.4f}, count={row['count']}\n")
            
            f.write("\n\nFiles generated:\n")
            f.write("- summary_statistics.csv: Detailed numerical statistics\n")
            f.write("- metric_comparison.png: Visual comparison plots\n")
            f.write("- detailed_report.txt: This report\n")
        
        logger.info(f"Generated detailed report: {report_file}")
    
    def run_analysis(self) -> bool:
        """Run the complete analysis pipeline."""
        logger.info("Starting benchmark analysis...")
        
        # Load data
        if not self.load_planner_results():
            logger.error("Failed to load planner results")
            return False
        
        # Generate summary statistics
        summary_df = self.generate_summary_statistics()
        
        # Create plots
        self.create_comparison_plots(summary_df)
        
        # Generate report
        self.generate_detailed_report(summary_df)
        
        logger.info("Analysis completed successfully!")
        logger.info(f"Results available in: {self.output_dir}")
        
        return True


def main():
    """Main function to run the analysis."""
    results_dir = "/home/chen/nuplan-devkit/planner_benchmark/results"
    
    if not os.path.exists(results_dir):
        logger.error(f"Results directory not found: {results_dir}")
        logger.info("Please run the benchmark first using run_simple_benchmark.sh")
        return
    
    analyzer = BenchmarkAnalyzer(results_dir)
    success = analyzer.run_analysis()
    
    if success:
        print(f"\nAnalysis complete! Check results in: {analyzer.output_dir}")
    else:
        print("Analysis failed. Check logs for details.")


if __name__ == "__main__":
    main()