#!/usr/bin/env python3
"""
Test suite for CSV Data Analyzer

Tests the CSV data structure analysis functionality.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import unittest
import pandas as pd
import numpy as np
import tempfile
import os
from pathlib import Path

# Add parent directory to path for imports
import sys
sys.path.append(str(Path(__file__).parent.parent))

from planning.csv_data_analyzer import CSVDataAnalyzer, CSVColumnMapping


class TestCSVDataAnalyzer(unittest.TestCase):
    """Test cases for CSV Data Analyzer."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create sample CSV data for testing
        self.sample_data = {
            'timestamp': [0.0, 0.1, 0.2, 0.3, 0.4],
            'scenario_id': ['scene_001', 'scene_001', 'scene_001', 'scene_001', 'scene_001'],
            'ego_x': [0.0, 1.0, 2.0, 3.0, 4.0],
            'ego_y': [0.0, 0.1, 0.2, 0.3, 0.4],
            'ego_heading': [0.0, 0.05, 0.1, 0.15, 0.2],
            'velocity_x': [10.0, 10.1, 10.2, 10.3, 10.4],
            'velocity_y': [0.0, 0.1, 0.2, 0.3, 0.4],
            'acceleration_x': [0.0, 0.1, 0.1, 0.1, 0.1],
            'acceleration_y': [0.0, 0.1, 0.1, 0.1, 0.1],
            'steering_angle': [0.0, 0.05, 0.1, 0.05, 0.0],
            'horizon_x_0': [1.0, 2.0, 3.0, 4.0, 5.0],
            'horizon_x_1': [2.0, 3.0, 4.0, 5.0, 6.0],
            'horizon_x_2': [3.0, 4.0, 5.0, 6.0, 7.0],
            'horizon_y_0': [0.1, 0.2, 0.3, 0.4, 0.5],
            'horizon_y_1': [0.2, 0.3, 0.4, 0.5, 0.6],
            'horizon_y_2': [0.3, 0.4, 0.5, 0.6, 0.7]
        }
        
        # Create temporary CSV file
        self.temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
        df = pd.DataFrame(self.sample_data)
        df.to_csv(self.temp_file.name, index=False)
        self.temp_file.close()
        
        # Initialize analyzer
        self.analyzer = CSVDataAnalyzer(self.temp_file.name)
    
    def tearDown(self):
        """Clean up test fixtures."""
        # Remove temporary file
        os.unlink(self.temp_file.name)
    
    def test_load_csv(self):
        """Test CSV loading functionality."""
        # Test successful loading
        self.assertTrue(self.analyzer.load_csv())
        self.assertIsNotNone(self.analyzer.df)
        self.assertEqual(len(self.analyzer.df), 5)
        self.assertEqual(len(self.analyzer.df.columns), 16)
        
        # Test with invalid file
        invalid_analyzer = CSVDataAnalyzer("nonexistent_file.csv")
        self.assertFalse(invalid_analyzer.load_csv())
    
    def test_analyze_structure(self):
        """Test structure analysis functionality."""
        self.analyzer.load_csv()
        results = self.analyzer.analyze_structure()
        
        # Check that all expected analysis sections are present
        expected_sections = [
            'basic_info', 'column_analysis', 'data_patterns',
            'temporal_analysis', 'trajectory_analysis', 'quality_assessment'
        ]
        
        for section in expected_sections:
            self.assertIn(section, results)
        
        # Check basic info
        basic_info = results['basic_info']
        self.assertEqual(basic_info['total_rows'], 5)
        self.assertEqual(basic_info['total_columns'], 16)
        self.assertFalse(basic_info['has_null_values'])
    
    def test_column_type_detection(self):
        """Test column type detection."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        
        column_analysis = self.analyzer.analysis_results['column_analysis']
        
        # Test timestamp detection
        timestamp_types = column_analysis['timestamp']['likely_types']
        self.assertIn('timestamp', timestamp_types)
        
        # Test position detection
        x_types = column_analysis['ego_x']['likely_types']
        self.assertIn('position_x', x_types)
        
        y_types = column_analysis['ego_y']['likely_types']
        self.assertIn('position_y', y_types)
        
        # Test heading detection
        heading_types = column_analysis['ego_heading']['likely_types']
        self.assertIn('heading', heading_types)
        
        # Test scenario ID detection
        scenario_types = column_analysis['scenario_id']['likely_types']
        self.assertIn('scenario_id', scenario_types)
    
    def test_temporal_analysis(self):
        """Test temporal structure analysis."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        
        temporal_analysis = self.analyzer.analysis_results['temporal_analysis']
        
        # Check time candidates
        time_candidates = temporal_analysis['time_candidates']
        self.assertIn('timestamp', time_candidates)
        
        # Check frequency analysis
        timestamp_analysis = temporal_analysis['timestamp_analysis']
        self.assertTrue(timestamp_analysis['successfully_parsed'])
        self.assertAlmostEqual(
            timestamp_analysis['frequency_analysis']['estimated_frequency_hz'], 
            10.0, 
            places=1
        )
    
    def test_trajectory_analysis(self):
        """Test trajectory structure analysis."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        
        trajectory_analysis = self.analyzer.analysis_results['trajectory_analysis']
        
        # Check trajectory sequences
        trajectory_sequences = trajectory_analysis['trajectory_sequences']
        self.assertIn('horizon_x', trajectory_sequences)
        self.assertIn('horizon_y', trajectory_sequences)
        
        # Check sequence properties
        horizon_x_seq = trajectory_sequences['horizon_x']
        self.assertEqual(horizon_x_seq['length'], 3)
        self.assertEqual(horizon_x_seq['indices'], [0, 1, 2])
    
    def test_mapping_generation(self):
        """Test mapping recommendation generation."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        mapping = self.analyzer.generate_mapping_recommendations()
        
        # Check recommended mappings
        self.assertEqual(mapping.timestamp_col, 'timestamp')
        self.assertEqual(mapping.scenario_id_col, 'scenario_id')
        self.assertEqual(mapping.x_col, 'ego_x')
        self.assertEqual(mapping.y_col, 'ego_y')
        self.assertEqual(mapping.heading_col, 'ego_heading')
        self.assertEqual(mapping.velocity_x_col, 'velocity_x')
        self.assertEqual(mapping.velocity_y_col, 'velocity_y')
        self.assertEqual(mapping.acceleration_x_col, 'acceleration_x')
        self.assertEqual(mapping.acceleration_y_col, 'acceleration_y')
        self.assertEqual(mapping.steering_angle_col, 'steering_angle')
        self.assertEqual(mapping.horizon_prefix, 'horizon_x')  # Should pick largest sequence
        self.assertEqual(mapping.horizon_length, 3)
    
    def test_quality_assessment(self):
        """Test data quality assessment."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        
        quality = self.analyzer.analysis_results['quality_assessment']
        
        # Check completeness (should be 100% for our test data)
        self.assertEqual(quality['completeness_ratio'], 1.0)
        
        # Check duplicates (should be 0 for our test data)
        self.assertEqual(quality['duplicate_rows'], 0)
        
        # Check outlier analysis exists
        self.assertIn('outlier_analysis', quality)
    
    def test_export_analysis(self):
        """Test analysis export functionality."""
        self.analyzer.load_csv()
        self.analyzer.analyze_structure()
        self.analyzer.generate_mapping_recommendations()
        
        # Export analysis
        export_path = self.analyzer.export_analysis()
        
        # Check file was created
        self.assertTrue(os.path.exists(export_path))
        
        # Check file contains valid JSON
        import json
        with open(export_path, 'r') as f:
            exported_data = json.load(f)
        
        self.assertIn('csv_file', exported_data)
        self.assertIn('analysis_results', exported_data)
        self.assertIn('recommended_mapping', exported_data)
        
        # Clean up
        os.unlink(export_path)


class TestCSVColumnMapping(unittest.TestCase):
    """Test cases for CSV Column Mapping."""
    
    def test_mapping_creation(self):
        """Test mapping object creation."""
        mapping = CSVColumnMapping(
            timestamp_col='time',
            x_col='pos_x',
            y_col='pos_y',
            heading_col='yaw'
        )
        
        self.assertEqual(mapping.timestamp_col, 'time')
        self.assertEqual(mapping.x_col, 'pos_x')
        self.assertEqual(mapping.y_col, 'pos_y')
        self.assertEqual(mapping.heading_col, 'yaw')
        
        # Test default values
        self.assertIsNone(mapping.scenario_id_col)
        self.assertIsNone(mapping.velocity_x_col)


if __name__ == '__main__':
    unittest.main()