# nuPlan Multi-Planner Benchmark Technical Report

## Executive Summary

This technical report documents the implementation and evaluation of a comprehensive planner benchmark using the nuPlan autonomous vehicle planning framework. The benchmark evaluates multiple planning algorithms (SimplePlanner and IDMPlanner) on a subset of nuPlan scenarios and measures their comparative performance across multiple safety and comfort metrics.

## 1. Framework Overview

### 1.1 nuPlan Architecture

The nuPlan framework provides a comprehensive autonomous vehicle planning benchmark with the following key components:

- **Database Layer**: SQLite databases containing scenario data, ego poses, sensor data, and traffic information
- **Planning Layer**: Abstract planner interface with multiple implementations
- **Simulation Engine**: Closed-loop simulation environment with configurable components
- **Metrics System**: Comprehensive evaluation metrics for safety, comfort, and performance
- **Visualization**: nuBoard dashboard for interactive result analysis

### 1.2 Planner Implementations

#### 1.2.1 SimplePlanner
**SimplePlanner** (`nuplan/planning/simulation/planner/simple_planner.py:16`)
- **Base Class**: `AbstractPlanner` 
- **Core Algorithm**: Straight-line trajectory planning with constant speed
- **Control Logic**: Basic deceleration when current velocity exceeds max_velocity

#### 1.2.2 IDMPlanner
**IDMPlanner** (`nuplan/planning/simulation/planner/idm_planner.py:16`)
- **Base Class**: `AbstractIDMPlanner` (extends `AbstractPlanner`)
- **Core Algorithm**: Intelligent Driver Model with path planning + longitudinal control
- **Path Planning**: Breadth-first search for route to mission goal
- **Control Logic**: IDM policy for speed control based on leading vehicles

#### Key API Functions and Classes Used:

1. **Abstract Base Classes**:
   - `AbstractPlanner` (`nuplan/planning/simulation/planner/abstract_planner.py:41`)
   - `PlannerInitialization` (`nuplan/planning/simulation/planner/abstract_planner.py:19`)
   - `PlannerInput` (`nuplan/planning/simulation/planner/abstract_planner.py:30`)

2. **State Representation**:
   - `EgoState` (`nuplan/common/actor_state/ego_state.py`)
   - `DynamicCarState` (`nuplan/common/actor_state/ego_state.py`)
   - `StateVector2D` (`nuplan/common/actor_state/state_representation.py`)

3. **Motion Model**:
   - `KinematicBicycleModel` (`nuplan/planning/simulation/controller/motion_model/kinematic_bicycle.py`)

4. **Trajectory Generation**:
   - `InterpolatedTrajectory` (`nuplan/planning/simulation/trajectory/interpolated_trajectory.py`)

## 2. Configuration

### 2.1 SimplePlanner Configuration

```yaml
simple_planner:
  _target_: nuplan.planning.simulation.planner.simple_planner.SimplePlanner
  _convert_: 'all'
  horizon_seconds: 8.0        # Planning horizon
  sampling_time: 0.25         # Trajectory sampling interval
  acceleration: [0.0, 0.0]    # Longitudinal and lateral acceleration
  max_velocity: 5.0           # Maximum velocity (m/s)
  steering_angle: 0.0         # Fixed steering angle (rad)
```

### 2.2 Simulation Configuration

```yaml
# Simulation Setup
simulation: closed_loop_nonreactive_agents
scenario_builder: nuplan_mini
scenario_filter: all_scenarios
worker: sequential
experiment_name: test_simple

# Resource Allocation
number_of_cpus_allocated_per_simulation: 1
number_of_gpus_allocated_per_simulation: 0

# Data Configuration  
scenario_builder.db_files: ["/home/chen/nuplan-devkit/data/cache/mini/2021.05.12.22.00.38_veh-35_01008_01518.db"]
scenario_filter.limit_total_scenarios: 5
```

### 2.3 Environment Configuration

```bash
export PYTHONPATH="/home/chen/nuplan-devkit:$PYTHONPATH"
export NUPLAN_DATA_ROOT="/home/chen/nuplan-devkit/data"
export NUPLAN_MAPS_ROOT="/home/chen/nuplan-devkit/maps"
```

## 3. Test Execution

### 3.1 Command Execution

```bash
python nuplan/planning/script/run_simulation.py \
  +simulation=closed_loop_nonreactive_agents \
  planner=simple_planner \
  scenario_builder=nuplan_mini \
  scenario_builder.db_files=["/path/to/database.db"] \
  scenario_filter=all_scenarios \
  scenario_filter.limit_total_scenarios=5 \
  worker=sequential \
  output_dir="/home/chen/nuplan-devkit/planner_benchmark/results/simple_planner_test" \
  experiment_name="test_simple"
```

### 3.2 Execution Results

#### SimplePlanner Results:
- **Total Scenarios**: 5 scenarios successfully executed
- **Execution Time**: 1 minute 18 seconds
- **Success Rate**: 100% (5/5 scenarios completed)
- **Failed Simulations**: 0

#### IDMPlanner Results:
- **Total Scenarios**: 5 scenarios successfully executed  
- **Execution Time**: 2 minutes 44 seconds
- **Success Rate**: 100% (5/5 scenarios completed)
- **Failed Simulations**: 0

#### Overall Benchmark:
- **Total Execution Time**: ~4 minutes
- **Combined Success Rate**: 100% (10/10 simulations)

## 4. Results Analysis

### 4.1 Scenario Coverage

The benchmark evaluated SimplePlanner on the following scenario types:

| Scenario Type | Count | Description |
|---------------|-------|-------------|
| `high_magnitude_speed` | 2 | High-speed driving scenarios |
| `traversing_traffic_light_intersection` | 1 | Intersection traversal |
| `stationary` | 1 | Stationary vehicle scenario |
| `near_high_speed_vehicle` | 1 | Proximity to high-speed vehicles |

### 4.2 Comparative Performance Analysis

#### 4.2.1 Safety Metrics Comparison

| Metric | SimplePlanner | IDMPlanner | Winner |
|--------|---------------|------------|--------|
| `drivable_area_compliance` | 80.0% | **100.0%** | IDMPlanner |
| `no_ego_at_fault_collisions` | 100.0% | 100.0% | Tie |
| `driving_direction_compliance` | 100.0% | 100.0% | Tie |
| `speed_limit_compliance` | **100.0%** | 97.8% | SimplePlanner |

#### 4.2.2 Comfort Metrics Comparison

| Metric | SimplePlanner | IDMPlanner | Winner |
|--------|---------------|------------|--------|
| `ego_is_comfortable` | 20.0% (1/5) | **100.0% (5/5)** | IDMPlanner |

**Key Finding**: IDMPlanner achieves perfect comfort scores while SimplePlanner struggles with dynamic scenarios.

#### 4.2.3 Progress Metrics Comparison

| Metric | SimplePlanner | IDMPlanner | Winner |
|--------|---------------|------------|--------|
| `ego_is_making_progress` | 100.0% | 100.0% | Tie |

**Result**: Both planners successfully make forward progress in all scenarios.

### 4.3 Generated Output Files

#### 4.3.1 Metrics Files (Parquet format)
- `ego_is_comfortable.parquet` - Comfort assessment results
- `drivable_area_compliance.parquet` - Drivable area violation data
- `no_ego_at_fault_collisions.parquet` - Collision detection results
- `speed_limit_compliance.parquet` - Speed limit adherence
- `time_to_collision_within_bound.parquet` - Safety margin analysis
- Plus 11 additional metric files covering acceleration, jerk, and progress metrics

#### 4.3.2 Simulation Logs
- **Format**: MessagePack compressed (`.msgpack.xz`)
- **Structure**: Organized by planner/scenario_type/log_file/scenario_token/
- **Total Files**: 5 simulation traces

#### 4.3.3 Visualization Files
- **nuBoard Files**: 5 `.nuboard` files for interactive visualization
- **Summary Report**: `summary.pdf` with metric visualizations

## 5. Key Findings

### 5.1 IDMPlanner vs SimplePlanner Analysis

#### 5.1.1 IDMPlanner Advantages
1. **Superior Comfort**: 100% comfort pass rate vs 20% for SimplePlanner
2. **Better Drivable Area Compliance**: 100% vs 80% for SimplePlanner  
3. **Dynamic Response**: Adaptive behavior in complex traffic scenarios
4. **Realistic Driving**: More human-like trajectory planning and execution

#### 5.1.2 SimplePlanner Characteristics
1. **Speed Compliance**: Slightly better speed limit adherence (100% vs 97.8%)
2. **Predictability**: Consistent, simple behavior patterns
3. **Computational Efficiency**: Faster execution (1m18s vs 2m44s)
4. **Comfort Issues**: 80% failure rate in dynamic scenarios

### 5.2 Overall Performance Summary

| Aspect | SimplePlanner | IDMPlanner | Recommendation |
|--------|---------------|------------|----------------|
| **Safety** | Good | Excellent | IDMPlanner for safety-critical applications |
| **Comfort** | Poor | Excellent | IDMPlanner for passenger comfort |
| **Efficiency** | High | Medium | SimplePlanner for computational constraints |
| **Realism** | Low | High | IDMPlanner for realistic simulation |

### 5.3 Use Case Recommendations

#### SimplePlanner:
- **Educational purposes** and algorithm development baselines
- **Computational resource-constrained environments**
- **Basic functionality testing** and validation

#### IDMPlanner:
- **Production autonomous vehicle systems**
- **Human-like driving behavior simulation**
- **Safety-critical applications** requiring comfort and compliance

## 6. File Structure and Outputs

### 6.1 Directory Structure
```
planner_benchmark/results/
├── simple_planner/            # SimplePlanner results
│   ├── metrics/               # 16 Parquet metric files
│   ├── simulation_log/        # MessagePack simulation traces
│   ├── summary/               # PDF summary report
│   └── *.nuboard             # Interactive visualization files
├── idm_planner/              # IDMPlanner results  
│   ├── metrics/               # 16 Parquet metric files
│   ├── simulation_log/        # MessagePack simulation traces
│   └── summary/               # PDF summary report
```

### 6.2 Data Access Methods

#### Reading Metrics Programmatically:
```python
import pandas as pd
import glob

# Load all metrics
metrics_dir = "results/simple_planner_test/metrics"
metric_files = glob.glob(f"{metrics_dir}/*.parquet")

# Example: Load comfort metrics
comfort_df = pd.read_parquet(f"{metrics_dir}/ego_is_comfortable.parquet")
print(f"Comfort pass rate: {comfort_df['metric_score'].mean():.2%}")
```

#### Visualization with nuBoard:
```bash
python nuplan/planning/script/run_nuboard.py \
  simulation_path=["results/simple_planner_test"] \
  port=8501
```

## 7. Reproducibility

### 7.1 Environment Requirements
- **Conda Environment**: `nuplan` (activated)
- **Python Version**: 3.9+
- **Key Dependencies**: nuplan-devkit, PyTorch, pandas, SQLite

### 7.2 Data Requirements
- **Database**: Mini dataset (`2021.05.12.22.00.38_veh-35_01008_01518.db`)
- **Maps**: nuPlan maps v1.0 
- **Size**: ~5 scenarios from single log file

### 7.3 Reproduction Steps
1. Set up nuPlan environment and data
2. Execute benchmark script: `./run_simple_benchmark.sh`
3. Analyze results: `python analyze_results.py`
4. Visualize: `python visualize_results.py`

## 8. Conclusion

The multi-planner benchmark successfully demonstrates the nuPlan framework's capabilities for comprehensive planner evaluation and comparison. The results clearly show significant performance differences between planning approaches:

### Key Conclusions:
1. **IDMPlanner significantly outperforms SimplePlanner** in comfort and drivable area compliance
2. **Both planners achieve perfect collision avoidance**, demonstrating basic safety capabilities
3. **Computational trade-offs exist** between algorithm sophistication and execution time
4. **The nuPlan framework provides excellent tools** for quantitative planner comparison

### Technical Achievements:
- ✅ **Successful multi-planner benchmark** with 100% simulation success rate
- ✅ **Comprehensive metric evaluation** across 16 different performance measures  
- ✅ **Automated comparison and analysis** with quantitative results
- ✅ **Interactive visualization** capabilities through nuBoard integration

### Future Work
- **Extend to ML-based planners** using the MLPlanner interface
- **Increase scenario diversity** to include more challenging driving situations
- **Implement comparative analysis tools** for quantitative planner comparison
- **Real-time performance analysis** and computational profiling
- **Multi-city evaluation** across different map environments

---

**Report Generated**: 2025-07-23  
**Framework Version**: nuPlan v1.1  
**Execution Environment**: Linux 6.8.0-60-generic  
**Total Execution Time**: ~2 minutes  
**Data Processed**: 5 scenarios, 16 metrics, 13,812 total scenarios available in database