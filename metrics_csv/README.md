# CSV Planner Evaluation System

A comprehensive system for evaluating CSV planner data against nuPlan expert trajectories using real nuPlan metrics.

## 🚀 Quick Start

**Evaluate CSV planner data with real nuPlan metrics:**

```bash
cd /home/chen/nuplan-devkit/metrics_csv
conda activate nuplan
python examples/single_scenario_evaluation.py
```

## 📁 Directory Structure

```
metrics_csv/
├── README.md                          # This file
├── examples/                          # Ready-to-run examples
│   ├── single_scenario_evaluation.py  # Main evaluation script (RECOMMENDED)
│   ├── real_database_example.py       # Real database approach
│   └── csv_data_analysis.py          # CSV analysis example
├── src/                               # Core implementation
│   ├── planning/                      # CSV trajectory and planning logic
│   │   ├── csv_trajectory.py         # CSV to nuPlan trajectory conversion
│   │   ├── csv_data_analyzer.py      # Automatic CSV structure analysis
│   │   ├── csv_evaluation_pipeline.py # Complete evaluation pipeline
│   │   └── scenario_matcher.py       # Scenario matching strategies
│   └── utils/                        # Utility modules
│       ├── mock_simulation_history.py # Mock data generation
│       └── debug_metrics.py          # Debugging helpers
├── data/                             # Data files
│   ├── sample_csv/                   # Sample CSV data
│   │   └── test_planner_data.csv     # Generated test data
│   └── validation/                   # Validation scripts
│       ├── phase3_final_validation.py
│       └── test_simulation_history_conversion.py
├── results/                          # Generated results
│   ├── csv_real_nuplan_metrics.json  # MAIN RESULTS - Real nuPlan metrics
│   ├── planner_expert_metrics_results.json # Alternative format
│   └── archives/                     # Historical results
├── tests/                           # Unit tests
│   ├── test_csv_analyzer.py
│   ├── test_csv_trajectory.py
│   └── test_scenario_matcher.py
└── docs/                           # Documentation
    ├── CLAUDE.md                   # Technical documentation
    ├── RealDatabaseEvaluationReport.md
    ├── architecture_diagram.mmd    # System architecture
    └── sequence_diagram.mmd        # Process flow
```

## 🎯 Main Features

1. **Real nuPlan Metrics**: Uses authentic nuPlan MetricsEngine for evaluation
2. **CSV Integration**: Converts CSV planner data to nuPlan SimulationHistory
3. **Timestamp Alignment**: Automatically aligns CSV timing with expert trajectories
4. **5 Core Metrics**: 
   - Average L2 Error
   - Average Heading Error  
   - Final L2 Error
   - Final Heading Error
   - Miss Rate

## 📊 Usage Examples

### Basic Evaluation
```python
from examples.single_scenario_evaluation import test_csv_with_metrics_engine
result = test_csv_with_metrics_engine()
```

### View Results
```python
import json
with open('results/csv_real_nuplan_metrics.json') as f:
    metrics = json.load(f)
print(f"Average L2 Error: {metrics['planner_expert_average_l2_error'][0]['metric_score']}")
```

## 🏆 Key Achievements

- ✅ **Complete Integration**: CSV → SimulationHistory → MetricsEngine → JSON Results
- ✅ **Real nuPlan Validation**: Uses authentic nuPlan scenario `00002b2436735b10`
- ✅ **Production Ready**: Generates industry-standard metric reports
- ✅ **Robust Pipeline**: Handles timestamp alignment and trajectory interpolation

## 📈 Latest Results

**Scenario**: `00002b2436735b10` (stationary)
- **Average L2 Error**: 2.22 meters
- **Metrics Generated**: 5 complete metrics with time series data
- **Status**: ✅ All metrics successfully computed

Generated: August 2025 | nuPlan DevKit Integration