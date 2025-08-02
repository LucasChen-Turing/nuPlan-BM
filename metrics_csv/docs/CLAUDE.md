# nuPlan Planning Modules - Structure and Functionality

This document provides an overview of the critical nuPlan planning modules: `nuplan/planning/metrics` and `nuplan/planning/simulation`. These modules are essential for autonomous vehicle planning evaluation and simulation.

## Table of Contents
- [nuplan/planning/metrics](#nuplanplanningmetrics)
- [nuplan/planning/simulation](#nuplanplanningsimulation)
- [Key Integration Points](#key-integration-points)
- [Development Guidelines](#development-guidelines)

## nuplan/planning/metrics

The metrics module provides a comprehensive framework for evaluating autonomous vehicle planning performance through various quantitative measures.

### Core Architecture

#### Abstract Base Classes
- **`AbstractMetricBuilder`** (`abstract_metric.py:9-54`): Base interface for all metrics
  - Defines `name`, `category` properties
  - `compute_score()`: Calculates final metric score
  - `compute()`: Main computation logic using simulation history

#### Metric Engine System
- **`MetricsEngine`** (`metric_engine.py:49-147`): Central orchestrator for metric computation
  - Manages collection of metric builders
  - Computes metrics in parallel with error handling
  - Serializes results to pickle files
  - Provides runtime performance tracking

#### Result Data Structures
- **`MetricStatistics`** (`metric_result.py:158-235`): Primary result container
  - Contains statistics, time series data, and final scores
  - Supports serialization/deserialization
  - Dataframe export capabilities
- **`Statistic`** (`metric_result.py:85-111`): Individual statistic values
- **`TimeSeries`** (`metric_result.py:115-154`): Time-based metric data
- **`MetricViolation`** (`metric_result.py:239-275`): Violation-specific results

#### Metric Base Classes
- **`MetricBase`** (`evaluation_metrics/base/metric_base.py:12-206`): Foundation for concrete metrics
  - Time series statistics computation (MAX, MIN, MEAN, P90)
  - Result construction helpers
  - Open-loop metric support with horizon analysis
- **`ViolationMetricBase`** (`evaluation_metrics/base/violation_metric_base.py:17-143`): For metrics tracking violations
  - Aggregates multiple violations into statistics
  - Computes violation-based scores (1 - violations/threshold)
  - Tracks number of violations, max/min/mean violation values
- **`WithinBoundMetricBase`** (`evaluation_metrics/base/within_bound_metric_base.py:13-148`): For threshold-based metrics
  - Checks if values stay within specified bounds
  - Returns boolean status and detailed statistics
  - Supports min/max threshold configuration

### Metric Categories

#### Common Metrics (`evaluation_metrics/common/`)
- **Safety**: `no_ego_at_fault_collisions.py`, `drivable_area_compliance.py`
- **Comfort**: `ego_acceleration.py`, `ego_jerk.py`, `ego_is_comfortable.py`
- **Progress**: `ego_is_making_progress.py`, `ego_progress_along_expert_route.py`
- **Compliance**: `speed_limit_compliance.py`, `driving_direction_compliance.py`
- **Accuracy**: `ego_expert_l2_error.py`, `planner_expert_*_error_within_bound.py`

#### Scenario-Dependent Metrics (`evaluation_metrics/scenario_dependent/`)
- **Traffic Rules**: `ego_stop_at_stop_line.py`

#### Aggregation (`aggregator/`)
- **`WeightedAverageMetricAggregator`**: Combines multiple metrics with weights

### Utility Functions (`utils/`)
- **`collision_utils.py`**: Collision detection algorithms
- **`expert_comparisons.py`**: Expert trajectory comparison tools
- **`route_extractor.py`**: Route analysis utilities
- **`state_extractors.py`**: State data extraction helpers
  - `extract_ego_acceleration()`: Extracts acceleration from ego states
  - `approximate_derivatives()`: Computes derivatives using Savitzky-Golay filter
  - `get_rectangle_corners()`: Gets vehicle footprint polygons
  - `calculate_ego_progress_to_goal()`: Measures progress toward mission goal

### Key Implementation Patterns

#### Pattern 1: WithinBound Metrics (e.g., `EgoAccelerationStatistics`)
```python
class EgoAccelerationStatistics(WithinBoundMetricBase):
    def compute(self, history, scenario):
        return self._compute_statistics(
            history=history,
            scenario=scenario,
            statistic_unit_name='meters_per_second_squared',
            extract_function=extract_ego_acceleration,
            extract_function_params={'acceleration_coordinate': 'magnitude'}
        )
```

#### Pattern 2: Violation Metrics (e.g., `NoEgoAtFaultCollisionStatistics`)
```python
# Detects collisions and classifies them by type
def find_new_collisions(ego_state, observation, collided_track_ids):
    # Returns collision type: STOPPED_EGO, ACTIVE_FRONT, ACTIVE_REAR, ACTIVE_LATERAL
    collision_type = _get_collision_type(ego_state, tracked_object)
    collision_data = CollisionData(delta_v, collision_type, object_type)
```

#### Pattern 3: Time Series Processing
- Extract relevant values from simulation history
- Compute statistics (MAX, MIN, MEAN, P90) 
- Create TimeSeries objects with timestamps and values
- Apply thresholds for within-bound checks or violation detection

## nuplan/planning/simulation

The simulation module provides a complete framework for running autonomous vehicle simulations with planners, observations, and control systems.

### Core Simulation Architecture

#### Main Simulation Class
- **`Simulation`** (`simulation.py:18-216`): Central simulation orchestrator
  - Manages simulation lifecycle (initialize, step, propagate)
  - Coordinates planner, controller, and observations
  - Maintains simulation history and buffer
  - Handles simulation state and termination conditions

#### Planner Framework
- **`AbstractPlanner`** (`planner/abstract_planner.py:41-123`): Base planner interface
  - `compute_planner_trajectory()`: Core trajectory computation
  - Runtime performance tracking
  - Observation type specification
  - Initialization support

#### Specialized Planners (`planner/`)
- **`IdmPlanner`**: Intelligent Driver Model implementation
- **`LogFuturePlanner`**: Uses logged future trajectories
- **`MLPlanner`**: Machine learning-based planning
- **`SimplePlanner`**: Basic rule-based planner

### Simulation Components

#### History Management
- **`SimulationHistory`** (`history/simulation_history.py:28-94`): Complete simulation record
  - Stores `SimulationHistorySample` sequences
  - Provides ego state extraction
  - Calculates simulation intervals
- **`SimulationHistoryBuffer`** (`history/simulation_history_buffer.py`): Rolling window of recent states
- **`SimulationHistorySample`** (`history/simulation_history.py:16-25`): Single timestep data

#### Observation System (`observation/`)
- **`AbstractObservation`** (`observation/abstract_observation.py:9-58`): Base observation interface
- **`TracksObservation`**: Tracked object observations
- **`LidarPc`**: Point cloud observations
- **`IdmAgents`**: Intelligent Driver Model agents

#### Control System (`controller/`)
- **`AbstractController`**: Base controller interface
- **`PerfectTracking`**: Ideal trajectory following
- **`LogPlayback`**: Replays logged trajectories
- **`TwoStageController`**: Hierarchical control approach

#### Motion Models (`controller/motion_model/`)
- **`KinematicBicycle`**: Bicycle motion model
- **`AbstractMotionModel`**: Motion model interface

### Callback System

#### Simulation Callbacks (`callback/`)
- **`AbstractCallback`** (`callback/abstract_callback.py:9-86`): Base callback interface
  - Hooks for initialization, step, planner, and simulation events
- **`MetricCallback`**: Computes metrics during simulation
- **`SerializationCallback`**: Saves simulation data
- **`VisualizationCallback`**: Real-time visualization
- **`TimingCallback`**: Performance measurement

#### Main Callbacks (`main_callback/`)
- **`MetricAggregatorCallback`**: Aggregates metrics across scenarios
- **`MetricFileCallback`**: Saves metric results to files
- **`CompletionCallback`**: Handles simulation completion

### Advanced Features

#### Trajectory Management (`trajectory/`)
- **`AbstractTrajectory`**: Base trajectory interface
- **`InterpolatedTrajectory`**: Smooth trajectory interpolation
- **`PredictedTrajectory`**: Future trajectory predictions

#### Path Planning (`path/`)
- **`InterpolatedPath`**: Path interpolation utilities
- **`Path`**: Basic path representation

#### Occupancy Mapping (`occupancy_map/`)
- **`AbstractOccupancyMap`**: Spatial occupancy interface
- **`GeopandasOccupancyMap`**: Geometric occupancy representation

#### Runner Framework (`runner/`)
- **`AbstractRunner`**: Base simulation runner
- **`SimulationsRunner`**: Batch simulation execution
- **`MetricRunner`**: Metric-focused simulation runs

## Key Integration Points

### Metrics ↔ Simulation Integration
1. **`SimulationHistory`** provides input data for metrics computation
2. **`MetricCallback`** enables real-time metric computation during simulation
3. **`MetricEngine`** processes simulation results for evaluation

### Data Flow
```
Scenario → Simulation → SimulationHistory → MetricsEngine → MetricStatistics
```

### Common Usage Patterns
1. **Evaluation Pipeline**: Simulation generates history → Metrics evaluate performance
2. **Real-time Monitoring**: Callbacks compute metrics during simulation
3. **Batch Processing**: Runner executes multiple simulations with metric aggregation

## Development Guidelines

### Adding New Metrics
1. Inherit from `MetricBase` or implement `AbstractMetricBuilder`
2. Implement `compute()` method using `SimulationHistory`
3. Define appropriate statistics and time series
4. Add tests following existing patterns

### Adding New Planners
1. Inherit from `AbstractPlanner`
2. Implement `compute_planner_trajectory()`
3. Specify required `observation_type()`
4. Handle initialization and runtime requirements

### Simulation Extensions
1. Use callback system for custom behavior
2. Extend observation types for new sensor modalities
3. Add controllers for different vehicle dynamics
4. Implement custom runners for specialized evaluation

### Testing Strategy
- Each module has comprehensive test coverage in `test/` directories
- JSON test fixtures in `test/json/` for metric validation
- Mock objects for isolated unit testing
- Integration tests for end-to-end workflows

### Performance Considerations
- Metrics engine includes runtime tracking and error handling
- Simulation buffers manage memory efficiently
- Parallel metric computation supported
- Serialization optimized for large datasets

## Metrics Evaluation Workflow

### Step-by-Step Evaluation Process

#### 1. Metric Setup
```python
# Create metric instances
metrics = [
    EgoAccelerationStatistics(name="ego_acceleration", category="comfort"),
    NoEgoAtFaultCollisionStatistics(name="no_collisions", category="safety"),
    EgoProgressAlongExpertRoute(name="progress", category="performance")
]

# Initialize metrics engine
engine = MetricsEngine(main_save_path=output_path, metrics=metrics)
```

#### 2. Simulation Execution
```python
# During simulation, history accumulates states
history.add_sample(SimulationHistorySample(
    iteration=iteration,
    ego_state=ego_state,
    trajectory=planned_trajectory,
    observation=observation,
    traffic_light_status=traffic_lights
))
```

#### 3. Metric Computation
```python
# After simulation completes
metric_results = engine.compute_metric_results(history, scenario)

# Each metric processes the history:
# - Extracts relevant data (positions, speeds, accelerations)
# - Computes time series statistics
# - Checks thresholds/violations
# - Generates MetricStatistics objects
```

#### 4. Result Structure
```python
MetricStatistics(
    metric_computator="ego_acceleration",
    name="ego_acceleration", 
    statistics=[
        Statistic(name="max_ego_acceleration", value=2.5, type=MAX, unit="m/s²"),
        Statistic(name="avg_ego_acceleration", value=1.2, type=MEAN, unit="m/s²"),
        Statistic(name="abs_ego_acceleration_within_bounds", value=True, type=BOOLEAN)
    ],
    time_series=TimeSeries(unit="m/s²", time_stamps=[...], values=[...]),
    metric_score=1.0,  # Computed by metric's compute_score() method
    metric_category="comfort"
)
```

#### 5. Aggregation and Storage
```python
# Combine results across scenarios
aggregator = WeightedAverageMetricAggregator(metric_weights={"safety": 0.5, "comfort": 0.3})
final_score = aggregator.aggregate_metrics(all_scenario_results)

# Save to files
engine.write_to_files(metric_files)  # Saves as pickle files
```

### Common Metric Categories

#### Safety Metrics
- **No Ego At Fault Collisions**: Detects and classifies collision types
- **Drivable Area Compliance**: Ensures ego stays in drivable regions
- **Speed Limit Compliance**: Checks adherence to speed regulations

#### Comfort Metrics  
- **Ego Acceleration/Jerk**: Monitors smooth driving behavior
- **Lateral/Longitudinal Dynamics**: Separate analysis of motion components
- **Ego Is Comfortable**: Composite comfort assessment

#### Performance Metrics
- **Ego Progress Along Expert Route**: Measures trajectory adherence
- **Ego Mean Speed**: Analyzes speed efficiency
- **Planner Expert L2 Error**: Compares against expert trajectories

#### Scenario-Specific Metrics
- **Ego Stop At Stop Line**: Traffic rule compliance at intersections
- **Time To Collision**: Safety margin analysis

### Integration with Simulation Pipeline

The metrics system integrates seamlessly with the simulation framework:

1. **Real-time Computation**: `MetricCallback` computes metrics during simulation
2. **Batch Processing**: `MetricRunner` processes multiple scenarios
3. **Result Aggregation**: `MetricAggregatorCallback` combines results
4. **Visualization**: Results compatible with nuBoard visualization

This comprehensive metrics framework enables quantitative evaluation of autonomous vehicle planning performance across safety, comfort, and efficiency dimensions.

This modular architecture enables flexible autonomous vehicle planning evaluation with comprehensive metrics and realistic simulation capabilities.