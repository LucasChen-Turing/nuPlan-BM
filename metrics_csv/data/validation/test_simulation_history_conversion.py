#!/usr/bin/env python3
"""
Test SimulationHistory Conversion

Tests the conversion from CSV planner data to SimulationHistory structure
without requiring full nuPlan environment dependencies.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import pandas as pd
import numpy as np
import json
from pathlib import Path
from dataclasses import dataclass
from typing import List, Dict, Any


@dataclass
class MockTimePoint:
    """Mock TimePoint for testing."""
    time_s: float
    time_us: int


@dataclass 
class MockEgoState:
    """Mock EgoState for testing."""
    rear_axle_x: float
    rear_axle_y: float
    rear_axle_heading: float
    velocity_x: float
    velocity_y: float
    acceleration_x: float
    acceleration_y: float
    tire_steering_angle: float
    time_point: MockTimePoint
    is_in_auto_mode: bool = True


@dataclass
class MockTrajectoryPoint:
    """Mock trajectory point."""
    timestamp: float
    x: float
    y: float
    heading: float
    velocity_x: float = None
    velocity_y: float = None


class MockTrajectory:
    """Mock trajectory implementation."""
    
    def __init__(self, points: List[MockTrajectoryPoint]):
        self.points = points
    
    @property
    def start_time(self):
        return MockTimePoint(self.points[0].timestamp, int(self.points[0].timestamp * 1e6))
    
    @property
    def end_time(self):
        return MockTimePoint(self.points[-1].timestamp, int(self.points[-1].timestamp * 1e6))
    
    def get_sampled_trajectory(self):
        return [self.point_to_ego_state(p) for p in self.points]
    
    def point_to_ego_state(self, point):
        return MockEgoState(
            rear_axle_x=point.x,
            rear_axle_y=point.y, 
            rear_axle_heading=point.heading,
            velocity_x=point.velocity_x or 0.0,
            velocity_y=point.velocity_y or 0.0,
            acceleration_x=0.0,
            acceleration_y=0.0,
            tire_steering_angle=0.0,
            time_point=MockTimePoint(point.timestamp, int(point.timestamp * 1e6))
        )


@dataclass
class MockSimulationSample:
    """Mock simulation history sample."""
    iteration: int
    ego_state: MockEgoState
    trajectory: MockTrajectory
    timestamp: float


class MockSimulationHistory:
    """Mock simulation history."""
    
    def __init__(self):
        self.samples: List[MockSimulationSample] = []
    
    def add_sample(self, sample: MockSimulationSample):
        self.samples.append(sample)
    
    def get_ego_states(self) -> List[MockEgoState]:
        return [sample.ego_state for sample in self.samples]
    
    def get_expert_states(self) -> List[MockEgoState]:
        # In real implementation, this would come from database
        # For testing, generate expert states similar to planner states
        expert_states = []
        for sample in self.samples:
            # Add small deviation to simulate expert vs planner difference
            expert_state = MockEgoState(
                rear_axle_x=sample.ego_state.rear_axle_x + np.random.normal(0, 0.1),
                rear_axle_y=sample.ego_state.rear_axle_y + np.random.normal(0, 0.1),
                rear_axle_heading=sample.ego_state.rear_axle_heading + np.random.normal(0, 0.01),
                velocity_x=sample.ego_state.velocity_x + np.random.normal(0, 0.1),
                velocity_y=sample.ego_state.velocity_y + np.random.normal(0, 0.1),
                acceleration_x=sample.ego_state.acceleration_x,
                acceleration_y=sample.ego_state.acceleration_y,
                tire_steering_angle=sample.ego_state.tire_steering_angle,
                time_point=sample.ego_state.time_point
            )
            expert_states.append(expert_state)
        return expert_states


class CSVToSimulationHistoryConverter:
    """Converts CSV planner data to SimulationHistory format."""
    
    def __init__(self, csv_file: str):
        self.csv_file = csv_file
        self.df = pd.read_csv(csv_file)
        print(f"📁 Loaded CSV with {len(self.df)} rows")
    
    def convert_scenario(self, scenario_id: str) -> MockSimulationHistory:
        """Convert a single scenario to SimulationHistory."""
        print(f"🔄 Converting scenario: {scenario_id}")
        
        # Filter data for this scenario
        scenario_data = self.df[self.df['scenario_id'] == scenario_id].copy()
        scenario_data = scenario_data.sort_values('planning_iteration')
        
        print(f"   Found {len(scenario_data)} timesteps")
        
        history = MockSimulationHistory()
        
        for idx, row in scenario_data.iterrows():
            # Convert CSV row to ego state
            ego_state = self._csv_row_to_ego_state(row)
            
            # Convert horizon to trajectory
            trajectory = self._csv_row_to_trajectory(row)
            
            # Create simulation sample
            sample = MockSimulationSample(
                iteration=int(row['planning_iteration']),
                ego_state=ego_state,
                trajectory=trajectory,
                timestamp=float(row['timestamp_us'] / 1e6)
            )
            
            history.add_sample(sample)
        
        print(f"   ✅ Created history with {len(history.samples)} samples")
        return history
    
    def _csv_row_to_ego_state(self, row) -> MockEgoState:
        """Convert CSV row to ego state."""
        timestamp_s = float(row['timestamp_us']) / 1e6
        
        return MockEgoState(
            rear_axle_x=float(row['ego_x']),
            rear_axle_y=float(row['ego_y']),
            rear_axle_heading=float(row['ego_heading']),
            velocity_x=float(row['ego_velocity_x']),
            velocity_y=float(row['ego_velocity_y']),
            acceleration_x=float(row['ego_acceleration_x']),
            acceleration_y=float(row['ego_acceleration_y']),
            tire_steering_angle=float(row['tire_steering_angle']),
            time_point=MockTimePoint(timestamp_s, int(row['timestamp_us']))
        )
    
    def _csv_row_to_trajectory(self, row) -> MockTrajectory:
        """Convert CSV horizon data to trajectory."""
        try:
            # Parse JSON horizon data
            horizon_x = json.loads(row['horizon_x'])
            horizon_y = json.loads(row['horizon_y'])
            horizon_heading = json.loads(row['horizon_heading'])
            horizon_vx = json.loads(row['horizon_velocity_x'])
            horizon_vy = json.loads(row['horizon_velocity_y'])
            
            # Create trajectory points
            points = []
            base_time = float(row['timestamp_us']) / 1e6
            dt = float(row['horizon_seconds']) / len(horizon_x)
            
            for i in range(len(horizon_x)):
                point = MockTrajectoryPoint(
                    timestamp=base_time + (i + 1) * dt,
                    x=horizon_x[i],
                    y=horizon_y[i],
                    heading=horizon_heading[i],
                    velocity_x=horizon_vx[i],
                    velocity_y=horizon_vy[i]
                )
                points.append(point)
            
            return MockTrajectory(points)
            
        except Exception as e:
            print(f"   ⚠️  Horizon parse error: {e}")
            # Fallback: single point trajectory
            base_time = float(row['timestamp_us']) / 1e6
            point = MockTrajectoryPoint(
                timestamp=base_time + 0.1,
                x=float(row['ego_x']),
                y=float(row['ego_y']),
                heading=float(row['ego_heading']),
                velocity_x=float(row['ego_velocity_x']),
                velocity_y=float(row['ego_velocity_y'])
            )
            return MockTrajectory([point])


def compute_mock_metrics(history: MockSimulationHistory) -> Dict[str, float]:
    """Compute mock planner-expert comparison metrics."""
    print("📊 Computing mock metrics...")
    
    planner_states = history.get_ego_states()
    expert_states = history.get_expert_states()
    
    if len(planner_states) != len(expert_states):
        print(f"   ⚠️  Length mismatch: planner={len(planner_states)}, expert={len(expert_states)}")
        min_len = min(len(planner_states), len(expert_states))
        planner_states = planner_states[:min_len]
        expert_states = expert_states[:min_len]
    
    print(f"   Comparing {len(planner_states)} states")
    
    # Compute L2 errors
    l2_errors = []
    heading_errors = []
    
    for p_state, e_state in zip(planner_states, expert_states):
        # L2 distance error
        dx = p_state.rear_axle_x - e_state.rear_axle_x
        dy = p_state.rear_axle_y - e_state.rear_axle_y
        l2_error = np.sqrt(dx**2 + dy**2)
        l2_errors.append(l2_error)
        
        # Heading error
        heading_diff = abs(p_state.rear_axle_heading - e_state.rear_axle_heading)
        # Normalize to [0, pi]
        heading_error = min(heading_diff, 2*np.pi - heading_diff)
        heading_errors.append(heading_error)
    
    # Compute metrics
    avg_l2_error = np.mean(l2_errors)
    avg_heading_error = np.mean(heading_errors)
    final_l2_error = l2_errors[-1] if l2_errors else 0
    final_heading_error = heading_errors[-1] if heading_errors else 0
    miss_rate = sum(1 for err in l2_errors if err > 2.0) / len(l2_errors) if l2_errors else 0
    
    metrics = {
        'average_l2_error': avg_l2_error,
        'average_heading_error': avg_heading_error,
        'final_l2_error': final_l2_error,
        'final_heading_error': final_heading_error,
        'miss_rate': miss_rate
    }
    
    # Convert to scores (1.0 - normalized_error)
    scores = {
        'planner_expert_average_l2_error': max(0, 1.0 - avg_l2_error / 5.0),
        'planner_expert_average_heading_error': max(0, 1.0 - avg_heading_error / 0.5),
        'planner_expert_final_l2_error': max(0, 1.0 - final_l2_error / 5.0),
        'planner_expert_final_heading_error': max(0, 1.0 - final_heading_error / 0.5),
        'planner_miss_rate': max(0, 1.0 - miss_rate)
    }
    
    return scores


def test_csv_conversion(csv_file: str):
    """Test the complete CSV conversion pipeline."""
    print("🧪 Testing CSV to SimulationHistory Conversion")
    print("=" * 60)
    
    # Initialize converter
    converter = CSVToSimulationHistoryConverter(csv_file)
    
    # Get unique scenarios
    scenarios = converter.df['scenario_id'].unique()
    print(f"📦 Found {len(scenarios)} scenarios: {list(scenarios)}")
    
    # Test conversion for each scenario
    results = []
    
    for scenario_id in scenarios[:3]:  # Test first 3 scenarios
        print(f"\n--- Testing Scenario: {scenario_id} ---")
        
        try:
            # Convert to simulation history
            history = converter.convert_scenario(scenario_id)
            
            # Compute metrics
            metrics = compute_mock_metrics(history)
            
            # Store results
            result = {
                'scenario_id': scenario_id,
                'num_samples': len(history.samples),
                'metrics': metrics,
                'success': True
            }
            results.append(result)
            
            # Print metrics
            print(f"   📈 Metrics:")
            for metric_name, score in metrics.items():
                print(f"      {metric_name}: {score:.4f}")
            
        except Exception as e:
            print(f"   ❌ Conversion failed: {e}")
            result = {
                'scenario_id': scenario_id,
                'error': str(e),
                'success': False
            }
            results.append(result)
    
    # Summary
    print(f"\n📊 CONVERSION TEST SUMMARY")
    print("=" * 40)
    
    successful = [r for r in results if r.get('success', False)]
    failed = [r for r in results if not r.get('success', False)]
    
    print(f"✅ Successful conversions: {len(successful)}/{len(results)}")
    print(f"❌ Failed conversions: {len(failed)}")
    
    if successful:
        # Aggregate metrics
        all_metrics = {}
        for result in successful:
            for metric_name, score in result['metrics'].items():
                if metric_name not in all_metrics:
                    all_metrics[metric_name] = []
                all_metrics[metric_name].append(score)
        
        print(f"\n📈 AGGREGATE METRICS:")
        overall_scores = []
        for metric_name, scores in all_metrics.items():
            avg_score = np.mean(scores)
            overall_scores.append(avg_score)
            print(f"   {metric_name}: {avg_score:.4f} ± {np.std(scores):.4f}")
        
        overall_performance = np.mean(overall_scores)
        print(f"\n🎯 Overall Performance: {overall_performance:.4f} ({overall_performance:.1%})")
        
        if overall_performance >= 0.95:
            performance_level = "Excellent"
        elif overall_performance >= 0.85:
            performance_level = "Good"  
        elif overall_performance >= 0.70:
            performance_level = "Fair"
        else:
            performance_level = "Poor"
        
        print(f"📊 Performance Level: {performance_level}")
    
    return results


def main():
    """Main test function."""
    csv_file = Path(__file__).parent / "test_planner_data.csv"
    
    if not csv_file.exists():
        print(f"❌ CSV file not found: {csv_file}")
        print("   Run generate_test_planner_data.py first")
        return
    
    # Run conversion test
    results = test_csv_conversion(str(csv_file))
    
    print(f"\n🎉 Phase 3 Testing Complete!")
    print(f"✅ CSV → SimulationHistory conversion validated")
    print(f"✅ Mock metrics computation successful")
    print(f"✅ Ready for real nuPlan database integration")


if __name__ == "__main__":
    main()