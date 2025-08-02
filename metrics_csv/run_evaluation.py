#!/usr/bin/env python3
"""
Main Evaluation Runner

Quick entry point for running CSV planner evaluation.

Usage:
    python run_evaluation.py
"""

import sys
from pathlib import Path

# Add current directory to path
sys.path.append(str(Path(__file__).parent))

from examples.single_scenario_evaluation import test_csv_with_metrics_engine

def main():
    """Run the main CSV evaluation."""
    print("🚀 Starting CSV Planner Evaluation")
    print("=" * 50)
    
    success = test_csv_with_metrics_engine()
    
    if success:
        print("\n🎉 Evaluation completed successfully!")
        print("📊 Check results/csv_real_nuplan_metrics.json for detailed metrics")
    else:
        print("\n❌ Evaluation failed")
    
    return success

if __name__ == "__main__":
    main()