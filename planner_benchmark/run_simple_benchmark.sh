#!/bin/bash
# Simple nuPlan Planner Benchmark Script
# This script runs simulations for different planners and compares results

# Activate nuplan conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate nuplan

# Set environment variables
export PYTHONPATH="/home/chen/nuplan-devkit:$PYTHONPATH"
export NUPLAN_DATA_ROOT="/home/chen/nuplan-devkit/data"
export NUPLAN_MAPS_ROOT="/home/chen/nuplan-devkit/maps"

# Create output directory
BENCHMARK_DIR="/home/chen/nuplan-devkit/planner_benchmark"
RESULTS_DIR="$BENCHMARK_DIR/results"
mkdir -p "$RESULTS_DIR"

cd /home/chen/nuplan-devkit

echo "Starting nuPlan Planner Benchmark..."
echo "Results will be saved to: $RESULTS_DIR"

# Function to run simulation for a planner
run_planner_simulation() {
    local planner_name=$1
    local planner_config=$2
    local output_subdir=$3
    
    echo "Running simulation for $planner_name..."
    
    python nuplan/planning/script/run_simulation.py \
        +simulation=closed_loop_nonreactive_agents \
        planner=$planner_config \
        scenario_builder=nuplan_mini \
        scenario_builder.db_files=["$NUPLAN_DATA_ROOT/cache/mini/2021.05.12.22.00.38_veh-35_01008_01518.db"] \
        scenario_filter=all_scenarios \
        scenario_filter.limit_total_scenarios=5 \
        worker=sequential \
        output_dir="$RESULTS_DIR/$output_subdir" \
        experiment_name="benchmark_$planner_name" \
        number_of_cpus_allocated_per_simulation=1 \
        number_of_gpus_allocated_per_simulation=0 \
        enable_simulation_progress_bar=True
    
    if [ $? -eq 0 ]; then
        echo "✓ Successfully completed simulation for $planner_name"
        return 0
    else
        echo "✗ Failed to run simulation for $planner_name"
        return 1
    fi
}

# Run benchmarks for different planners
echo "============================================"
echo "Running SimplePlanner benchmark..."
echo "============================================"
run_planner_simulation "SimplePlanner" "simple_planner" "simple_planner"

echo ""
echo "============================================"
echo "Running IDMPlanner benchmark..."
echo "============================================"
run_planner_simulation "IDMPlanner" "idm_planner" "idm_planner"

echo ""
echo "============================================"
echo "Benchmark completed!"
echo "============================================"

# List results
echo "Results directory structure:"
find "$RESULTS_DIR" -type f -name "*.parquet" | head -10

echo ""
echo "To visualize results, run:"
echo "python nuplan/planning/script/run_nuboard.py simulation_path='$RESULTS_DIR'"