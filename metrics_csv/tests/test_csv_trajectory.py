#!/usr/bin/env python3
"""
Test suite for CSV Trajectory Implementation

Tests the CSV trajectory conversion functionality.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import unittest
import numpy as np
import pandas as pd
from pathlib import Path

# Add parent directory to path for imports
import sys
sys.path.append(str(Path(__file__).parent.parent))

from planning.csv_trajectory import CSVTrajectory, CSVTrajectoryPoint, CSVEgoStateConverter
from planning.csv_data_analyzer import CSVColumnMapping
from nuplan.common.actor_state.state_representation import TimePoint


class TestCSVTrajectoryPoint(unittest.TestCase):
    """Test cases for CSV Trajectory Point."""
    
    def test_trajectory_point_creation(self):
        """Test trajectory point creation."""
        point = CSVTrajectoryPoint(
            timestamp=1.0,
            x=10.0,
            y=5.0,
            heading=0.5,
            velocity_x=15.0,
            velocity_y=0.1,
            acceleration_x=0.5,
            acceleration_y=0.0,
            steering_angle=0.1
        )
        
        self.assertEqual(point.timestamp, 1.0)
        self.assertEqual(point.x, 10.0)
        self.assertEqual(point.y, 5.0)
        self.assertEqual(point.heading, 0.5)
        self.assertEqual(point.velocity_x, 15.0)
        self.assertEqual(point.velocity_y, 0.1)
        self.assertEqual(point.acceleration_x, 0.5)
        self.assertEqual(point.acceleration_y, 0.0)
        self.assertEqual(point.steering_angle, 0.1)
    
    def test_trajectory_point_optional_fields(self):
        """Test trajectory point with optional fields."""
        point = CSVTrajectoryPoint(
            timestamp=1.0,
            x=10.0,
            y=5.0,
            heading=0.5
        )
        
        self.assertEqual(point.timestamp, 1.0)
        self.assertEqual(point.x, 10.0)
        self.assertEqual(point.y, 5.0)
        self.assertEqual(point.heading, 0.5)
        self.assertIsNone(point.velocity_x)
        self.assertIsNone(point.velocity_y)
        self.assertIsNone(point.acceleration_x)
        self.assertIsNone(point.acceleration_y)
        self.assertIsNone(point.steering_angle)


class TestCSVTrajectory(unittest.TestCase):
    """Test cases for CSV Trajectory."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create sample trajectory points
        self.trajectory_points = [
            CSVTrajectoryPoint(
                timestamp=0.0, x=0.0, y=0.0, heading=0.0,
                velocity_x=10.0, velocity_y=0.0,
                acceleration_x=1.0, acceleration_y=0.0,
                steering_angle=0.0
            ),
            CSVTrajectoryPoint(
                timestamp=0.1, x=1.0, y=0.1, heading=0.05,
                velocity_x=10.1, velocity_y=0.1,
                acceleration_x=1.0, acceleration_y=0.1,
                steering_angle=0.05
            ),
            CSVTrajectoryPoint(
                timestamp=0.2, x=2.0, y=0.2, heading=0.1,
                velocity_x=10.2, velocity_y=0.2,
                acceleration_x=1.0, acceleration_y=0.1,
                steering_angle=0.1
            )
        ]
        
        self.trajectory = CSVTrajectory(self.trajectory_points)
    
    def test_trajectory_creation(self):
        """Test trajectory creation."""
        self.assertEqual(len(self.trajectory.get_sampled_trajectory()), 3)
        
        # Test time properties
        self.assertEqual(self.trajectory.start_time.time_s, 0.0)
        self.assertEqual(self.trajectory.end_time.time_s, 0.2)
        self.assertAlmostEqual(self.trajectory.duration, 0.2, places=3)
    
    def test_empty_trajectory_error(self):
        """Test that empty trajectory raises error."""
        with self.assertRaises(ValueError):
            CSVTrajectory([])
    
    def test_get_state_at_time(self):
        """Test getting state at specific time."""
        # Test exact time match
        state_0 = self.trajectory.get_state_at_time(TimePoint(0))
        self.assertAlmostEqual(state_0.rear_axle.x, 0.0, places=3)
        self.assertAlmostEqual(state_0.rear_axle.y, 0.0, places=3)
        
        state_2 = self.trajectory.get_state_at_time(TimePoint(int(0.2 * 1e6)))
        self.assertAlmostEqual(state_2.rear_axle.x, 2.0, places=3)
        self.assertAlmostEqual(state_2.rear_axle.y, 0.2, places=3)
        
        # Test interpolation
        state_mid = self.trajectory.get_state_at_time(TimePoint(int(0.1 * 1e6)))
        self.assertAlmostEqual(state_mid.rear_axle.x, 1.0, places=3)
        self.assertAlmostEqual(state_mid.rear_axle.y, 0.1, places=3)
    
    def test_get_state_at_times(self):
        """Test getting states at multiple times."""
        time_points = [TimePoint(0), TimePoint(int(0.1 * 1e6)), TimePoint(int(0.2 * 1e6))]
        states = self.trajectory.get_state_at_times(time_points)
        
        self.assertEqual(len(states), 3)
        self.assertAlmostEqual(states[0].rear_axle.x, 0.0, places=3)
        self.assertAlmostEqual(states[1].rear_axle.x, 1.0, places=3)
        self.assertAlmostEqual(states[2].rear_axle.x, 2.0, places=3)
    
    def test_out_of_range_time(self):
        """Test error for out-of-range time query."""
        with self.assertRaises(AssertionError):
            self.trajectory.get_state_at_time(TimePoint(int(1.0 * 1e6)))  # Beyond end time
        
        with self.assertRaises(AssertionError):
            self.trajectory.get_state_at_time(TimePoint(int(-1.0 * 1e6)))  # Before start time
    
    def test_is_in_range(self):
        """Test time range checking."""
        self.assertTrue(self.trajectory.is_in_range(TimePoint(0)))
        self.assertTrue(self.trajectory.is_in_range(TimePoint(int(0.1 * 1e6))))
        self.assertTrue(self.trajectory.is_in_range(TimePoint(int(0.2 * 1e6))))
        
        self.assertFalse(self.trajectory.is_in_range(TimePoint(int(-0.1 * 1e6))))
        self.assertFalse(self.trajectory.is_in_range(TimePoint(int(0.3 * 1e6))))


class TestCSVEgoStateConverter(unittest.TestCase):
    """Test cases for CSV Ego State Converter."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create column mapping
        self.mapping = CSVColumnMapping(
            timestamp_col='timestamp',
            scenario_id_col='scenario_id',
            x_col='ego_x',
            y_col='ego_y',
            heading_col='ego_heading',
            velocity_x_col='velocity_x',
            velocity_y_col='velocity_y',
            acceleration_x_col='acceleration_x',
            acceleration_y_col='acceleration_y',
            steering_angle_col='steering_angle',
            horizon_prefix='horizon'
        )
        
        self.converter = CSVEgoStateConverter(self.mapping)
        
        # Create sample CSV row
        self.csv_row = {
            'timestamp': 1.5,
            'scenario_id': 'test_scenario',
            'ego_x': 10.0,
            'ego_y': 5.0,
            'ego_heading': 0.5,
            'velocity_x': 15.0,
            'velocity_y': 0.5,
            'acceleration_x': 1.0,
            'acceleration_y': 0.1,
            'steering_angle': 0.1,
            'horizon_x_0': 11.0,
            'horizon_x_1': 12.0,
            'horizon_x_2': 13.0,
            'horizon_y_0': 5.1,
            'horizon_y_1': 5.2,
            'horizon_y_2': 5.3,
            'horizon_heading_0': 0.51,
            'horizon_heading_1': 0.52,
            'horizon_heading_2': 0.53
        }
    
    def test_convert_row_to_ego_state(self):
        """Test converting CSV row to ego state."""
        ego_state = self.converter.convert_row_to_ego_state(self.csv_row)
        
        # Check position
        self.assertAlmostEqual(ego_state.rear_axle.x, 10.0, places=3)
        self.assertAlmostEqual(ego_state.rear_axle.y, 5.0, places=3)
        self.assertAlmostEqual(ego_state.rear_axle.heading, 0.5, places=3)
        
        # Check velocities
        velocity = ego_state.dynamic_car_state.rear_axle_velocity_2d
        self.assertAlmostEqual(velocity.x, 15.0, places=3)
        self.assertAlmostEqual(velocity.y, 0.5, places=3)
        
        # Check accelerations
        acceleration = ego_state.dynamic_car_state.rear_axle_acceleration_2d
        self.assertAlmostEqual(acceleration.x, 1.0, places=3)
        self.assertAlmostEqual(acceleration.y, 0.1, places=3)
        
        # Check steering
        self.assertAlmostEqual(ego_state.tire_steering_angle, 0.1, places=3)
        
        # Check timestamp
        self.assertEqual(ego_state.time_point.time_s, 1.5)
        
        # Check autonomous mode
        self.assertTrue(ego_state.is_in_auto_mode)
    
    def test_convert_row_pandas_series(self):
        """Test converting pandas Series to ego state."""
        series_row = pd.Series(self.csv_row)
        ego_state = self.converter.convert_row_to_ego_state(series_row)
        
        self.assertAlmostEqual(ego_state.rear_axle.x, 10.0, places=3)
        self.assertAlmostEqual(ego_state.rear_axle.y, 5.0, places=3)
        self.assertEqual(ego_state.time_point.time_s, 1.5)
    
    def test_convert_horizon_to_trajectory(self):
        """Test converting horizon data to trajectory."""
        trajectory = self.converter.convert_horizon_to_trajectory(
            self.csv_row, 
            base_timestamp=1.5, 
            dt=0.1
        )
        
        # Check trajectory properties
        sampled_states = trajectory.get_sampled_trajectory()
        self.assertEqual(len(sampled_states), 3)
        
        # Check first point
        self.assertAlmostEqual(sampled_states[0].rear_axle.x, 11.0, places=3)
        self.assertAlmostEqual(sampled_states[0].rear_axle.y, 5.1, places=3)
        self.assertAlmostEqual(sampled_states[0].rear_axle.heading, 0.51, places=3)
        
        # Check timestamps
        self.assertEqual(sampled_states[0].time_point.time_s, 1.5)
        self.assertEqual(sampled_states[1].time_point.time_s, 1.6)
        self.assertEqual(sampled_states[2].time_point.time_s, 1.7)
    
    def test_missing_columns_handled(self):
        """Test handling of missing optional columns."""
        # Create row with missing optional fields
        minimal_row = {
            'timestamp': 1.0,
            'ego_x': 5.0,
            'ego_y': 2.0,
            'ego_heading': 0.2
        }
        
        ego_state = self.converter.convert_row_to_ego_state(minimal_row)
        
        # Check that missing fields default to 0
        velocity = ego_state.dynamic_car_state.rear_axle_velocity_2d
        self.assertEqual(velocity.x, 0.0)
        self.assertEqual(velocity.y, 0.0)
        
        acceleration = ego_state.dynamic_car_state.rear_axle_acceleration_2d
        self.assertEqual(acceleration.x, 0.0)
        self.assertEqual(acceleration.y, 0.0)
        
        self.assertEqual(ego_state.tire_steering_angle, 0.0)
    
    def test_no_horizon_prefix_error(self):
        """Test error when no horizon prefix is configured."""
        mapping_no_horizon = CSVColumnMapping(
            x_col='ego_x',
            y_col='ego_y',
            heading_col='ego_heading'
        )
        
        converter_no_horizon = CSVEgoStateConverter(mapping_no_horizon)
        
        with self.assertRaises(ValueError):
            converter_no_horizon.convert_horizon_to_trajectory(
                self.csv_row, 
                base_timestamp=1.0
            )


if __name__ == '__main__':
    unittest.main()