#!/usr/bin/env python3
"""
Test suite for Scenario Matcher

Tests the scenario matching functionality.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import unittest
import pandas as pd
import numpy as np
from pathlib import Path
from unittest.mock import Mock, MagicMock

# Add parent directory to path for imports
import sys
sys.path.append(str(Path(__file__).parent.parent))

from planning.scenario_matcher import ScenarioMatcher, ScenarioMatch, MatchingConfig
from planning.csv_data_analyzer import CSVColumnMapping


class TestScenarioMatcher(unittest.TestCase):
    """Test cases for Scenario Matcher."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create sample CSV data
        self.csv_data = pd.DataFrame({
            'timestamp': [0.0, 0.1, 0.2, 0.3, 0.4],
            'scenario_id': ['scene_001', 'scene_001', 'scene_001', 'scene_001', 'scene_001'],
            'ego_x': [0.0, 1.0, 2.0, 3.0, 4.0],
            'ego_y': [0.0, 0.1, 0.2, 0.3, 0.4],
            'ego_heading': [0.0, 0.05, 0.1, 0.15, 0.2]
        })
        
        # Create column mapping
        self.column_mapping = CSVColumnMapping(
            timestamp_col='timestamp',
            scenario_id_col='scenario_id',
            x_col='ego_x',
            y_col='ego_y',
            heading_col='ego_heading'
        )
        
        # Create matching config
        self.config = MatchingConfig()
        
        # Initialize matcher
        self.matcher = ScenarioMatcher(
            csv_data=self.csv_data,
            column_mapping=self.column_mapping,
            config=self.config
        )
    
    def create_mock_scenario(self, scenario_name: str, scenario_type: str = "test_type"):
        """Create a mock nuPlan scenario for testing."""
        mock_scenario = Mock()
        mock_scenario.scenario_name = scenario_name
        mock_scenario.scenario_type = scenario_type
        
        # Create mock expert trajectory
        mock_states = []
        for i in range(5):
            mock_state = Mock()
            mock_state.time_point.time_s = i * 0.1
            mock_state.rear_axle.x = i * 1.0
            mock_state.rear_axle.y = i * 0.1
            mock_states.append(mock_state)
        
        mock_scenario.get_expert_ego_trajectory.return_value = iter(mock_states)
        
        return mock_scenario
    
    def test_matcher_initialization(self):
        """Test matcher initialization."""
        self.assertEqual(len(self.matcher.csv_scenarios), 1)
        self.assertIn('scene_001', self.matcher.csv_scenarios)
        self.assertEqual(self.matcher.csv_scenarios['scene_001'], [0, 1, 2, 3, 4])
    
    def test_prepare_csv_data_no_scenario_id(self):
        """Test CSV preparation when no scenario ID column exists."""
        # Create CSV without scenario ID
        csv_no_id = pd.DataFrame({
            'timestamp': [0.0, 0.1, 0.2],
            'ego_x': [0.0, 1.0, 2.0],
            'ego_y': [0.0, 0.1, 0.2]
        })
        
        mapping_no_id = CSVColumnMapping(
            timestamp_col='timestamp',
            x_col='ego_x',
            y_col='ego_y'
        )
        
        matcher = ScenarioMatcher(
            csv_data=csv_no_id,
            column_mapping=mapping_no_id
        )
        
        self.assertEqual(len(matcher.csv_scenarios), 1)
        self.assertIn('unknown_scenario', matcher.csv_scenarios)
    
    def test_match_by_scenario_id_exact_match(self):
        """Test exact scenario ID matching."""
        # Create mock scenarios
        scenarios = [
            self.create_mock_scenario('scene_001'),
            self.create_mock_scenario('scene_002'),
        ]
        
        matches = self.matcher.match_by_scenario_id(scenarios)
        
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].scenario_id, 'scene_001')
        self.assertEqual(matches[0].scenario_name, 'scene_001')
        self.assertEqual(matches[0].matching_method, 'exact_id_match')
        self.assertEqual(matches[0].matching_confidence, 1.0)
    
    def test_match_by_scenario_id_prefix_match(self):
        """Test prefix scenario ID matching."""
        # Create mock scenarios with different names
        scenarios = [
            self.create_mock_scenario('scene_001_extended'),
            self.create_mock_scenario('other_scenario'),
        ]
        
        matches = self.matcher.match_by_scenario_id(scenarios)
        
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].scenario_id, 'scene_001')
        self.assertEqual(matches[0].scenario_name, 'scene_001_extended')
        self.assertEqual(matches[0].matching_method, 'prefix_id_match')
        self.assertGreater(matches[0].matching_confidence, 0.5)
    
    def test_match_by_scenario_id_no_match(self):
        """Test scenario ID matching with no matches."""
        # Create mock scenarios with unrelated names
        scenarios = [
            self.create_mock_scenario('different_001'),
            self.create_mock_scenario('another_002'),
        ]
        
        matches = self.matcher.match_by_scenario_id(scenarios)
        
        self.assertEqual(len(matches), 0)
    
    def test_match_by_temporal_alignment(self):
        """Test temporal alignment matching."""
        # Create mock scenario with similar duration
        scenarios = [self.create_mock_scenario('test_scenario')]
        
        matches = self.matcher.match_by_temporal_alignment(scenarios)
        
        # Should find a match based on similar durations
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].matching_method, 'temporal_alignment')
        self.assertGreater(matches[0].matching_confidence, 0.0)
    
    def test_match_by_temporal_alignment_no_timestamp(self):
        """Test temporal matching when no timestamp column exists."""
        # Create matcher without timestamp column
        mapping_no_time = CSVColumnMapping(
            scenario_id_col='scenario_id',
            x_col='ego_x',
            y_col='ego_y'
        )
        
        matcher = ScenarioMatcher(
            csv_data=self.csv_data,
            column_mapping=mapping_no_time
        )
        
        scenarios = [self.create_mock_scenario('test_scenario')]
        matches = matcher.match_by_temporal_alignment(scenarios)
        
        self.assertEqual(len(matches), 0)
    
    def test_match_by_spatial_trajectory(self):
        """Test spatial trajectory matching."""
        scenarios = [self.create_mock_scenario('test_scenario')]
        
        matches = self.matcher.match_by_spatial_trajectory(scenarios)
        
        # Should find a match based on similar trajectories
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].matching_method, 'spatial_trajectory')
        self.assertGreater(matches[0].matching_confidence, 0.0)
    
    def test_calculate_trajectory_similarity(self):
        """Test trajectory similarity calculation."""
        # Create identical trajectories
        csv_x = np.array([0.0, 1.0, 2.0, 3.0])
        csv_y = np.array([0.0, 0.1, 0.2, 0.3])
        expert_x = np.array([0.0, 1.0, 2.0, 3.0])
        expert_y = np.array([0.0, 0.1, 0.2, 0.3])
        
        similarity = self.matcher._calculate_trajectory_similarity(
            csv_x, csv_y, expert_x, expert_y
        )
        
        # Identical trajectories should have high similarity
        self.assertGreater(similarity, 0.9)
        
        # Create very different trajectories
        different_x = np.array([100.0, 101.0, 102.0, 103.0])
        different_y = np.array([100.0, 100.1, 100.2, 100.3])
        
        similarity_different = self.matcher._calculate_trajectory_similarity(
            csv_x, csv_y, different_x, different_y
        )
        
        # Different trajectories should have low similarity
        self.assertLess(similarity_different, 0.1)
    
    def test_find_best_matches(self):
        """Test finding best matches using all strategies."""
        scenarios = [
            self.create_mock_scenario('scene_001'),  # Should match by ID
            self.create_mock_scenario('other_scenario'),
        ]
        
        matches = self.matcher.find_best_matches(scenarios)
        
        # Should find the ID-based match
        self.assertEqual(len(matches), 1)
        self.assertEqual(matches[0].scenario_name, 'scene_001')
        self.assertEqual(matches[0].matching_method, 'exact_id_match')
    
    def test_select_best_matches_no_duplicates(self):
        """Test that best match selection avoids duplicates."""
        # Create multiple matches for same scenario
        matches = [
            ScenarioMatch(
                scenario_id='csv_001',
                scenario_name='db_scenario_1',
                csv_rows=[0, 1, 2],
                expert_trajectory_length=3,
                time_alignment_offset=0.0,
                matching_confidence=0.8,
                matching_method='spatial_trajectory',
                metadata={}
            ),
            ScenarioMatch(
                scenario_id='csv_001',
                scenario_name='db_scenario_1',
                csv_rows=[0, 1, 2],
                expert_trajectory_length=3,
                time_alignment_offset=0.0,
                matching_confidence=0.9,
                matching_method='exact_id_match',
                metadata={}
            )
        ]
        
        best_matches = self.matcher._select_best_matches(matches)
        
        # Should select only one match (the better one)
        self.assertEqual(len(best_matches), 1)
        self.assertEqual(best_matches[0].matching_method, 'exact_id_match')
        self.assertEqual(best_matches[0].matching_confidence, 0.9)
    
    def test_confidence_threshold_filtering(self):
        """Test that low-confidence matches are filtered out."""
        # Set high confidence threshold
        config = MatchingConfig(min_confidence_threshold=0.95)
        
        matcher = ScenarioMatcher(
            csv_data=self.csv_data,
            column_mapping=self.column_mapping,
            config=config
        )
        
        # Create scenario that would produce medium confidence match
        scenarios = [self.create_mock_scenario('scene_001_somewhat_different')]
        
        matches = matcher.match_by_scenario_id(scenarios)
        
        # Should reject the match due to low confidence
        self.assertEqual(len(matches), 0)


class TestMatchingConfig(unittest.TestCase):
    """Test cases for Matching Config."""
    
    def test_default_config(self):
        """Test default configuration values."""
        config = MatchingConfig()
        
        self.assertEqual(config.max_time_difference, 2.0)
        self.assertEqual(config.max_spatial_distance, 50.0)
        self.assertEqual(config.min_confidence_threshold, 0.6)
        self.assertTrue(config.use_scenario_id_matching)
    
    def test_custom_config(self):
        """Test custom configuration values."""
        config = MatchingConfig(
            max_time_difference=5.0,
            max_spatial_distance=100.0,
            min_confidence_threshold=0.8,
            use_scenario_id_matching=False
        )
        
        self.assertEqual(config.max_time_difference, 5.0)
        self.assertEqual(config.max_spatial_distance, 100.0)
        self.assertEqual(config.min_confidence_threshold, 0.8)
        self.assertFalse(config.use_scenario_id_matching)


class TestScenarioMatch(unittest.TestCase):
    """Test cases for Scenario Match dataclass."""
    
    def test_scenario_match_creation(self):
        """Test scenario match object creation."""
        match = ScenarioMatch(
            scenario_id='test_id',
            scenario_name='test_name',
            csv_rows=[0, 1, 2],
            expert_trajectory_length=5,
            time_alignment_offset=1.5,
            matching_confidence=0.85,
            matching_method='temporal_alignment',
            metadata={'scenario_type': 'test_type'}
        )
        
        self.assertEqual(match.scenario_id, 'test_id')
        self.assertEqual(match.scenario_name, 'test_name')
        self.assertEqual(match.csv_rows, [0, 1, 2])
        self.assertEqual(match.expert_trajectory_length, 5)
        self.assertEqual(match.time_alignment_offset, 1.5)
        self.assertEqual(match.matching_confidence, 0.85)
        self.assertEqual(match.matching_method, 'temporal_alignment')
        self.assertEqual(match.metadata['scenario_type'], 'test_type')


if __name__ == '__main__':
    unittest.main()