#!/bin/bash
# Simple nuBoard launcher script for benchmark results

# Activate nuplan conda environment
source ~/miniconda3/etc/profile.d/conda.sh
conda activate nuplan

# Set environment variables
export PYTHONPATH="/home/chen/nuplan-devkit:$PYTHONPATH"
export NUPLAN_DATA_ROOT="/home/chen/nuplan-devkit/data"
export NUPLAN_MAPS_ROOT="/home/chen/nuplan-devkit/maps"

# Change to nuplan directory
cd /home/chen/nuplan-devkit

# Launch nuBoard
echo "Launching nuBoard for SimplePlanner benchmark results..."
echo "nuBoard will be available at: http://localhost:8501"
echo "Press Ctrl+C to stop nuBoard"
echo ""

python nuplan/planning/script/run_nuboard.py \
    simulation_path="/home/chen/nuplan-devkit/planner_benchmark/results/simple_planner" \
    port_number=8501 \
    scenario_builder.db_files=["/home/chen/nuplan-devkit/data/cache/mini/2021.05.12.22.00.38_veh-35_01008_01518.db"]