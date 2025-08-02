#!/usr/bin/env python3
"""
Test CSV Analysis with Generated Data

Tests the CSV analysis and conversion pipeline with our generated planner data.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import sys
import pandas as pd
import numpy as np
import json
from pathlib import Path

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

def analyze_csv_simple(csv_file):
    """Simple CSV analysis without nuPlan dependencies."""
    print("🔍 Analyzing CSV Structure")
    print("=" * 50)
    
    df = pd.read_csv(csv_file)
    
    print(f"📊 Basic Info:")
    print(f"   Rows: {len(df)}")
    print(f"   Columns: {len(df.columns)}")
    print(f"   Memory: {df.memory_usage(deep=True).sum() / 1024:.1f} KB")
    print(f"   Completeness: {((df.size - df.isnull().sum().sum()) / df.size):.1%}")
    
    print(f"\n📋 Column Analysis:")
    for col in df.columns:
        dtype = str(df[col].dtype)
        unique_vals = df[col].nunique()
        null_count = df[col].isnull().sum()
        sample_val = df[col].iloc[0] if len(df) > 0 else "N/A"
        
        # Truncate long values
        if isinstance(sample_val, str) and len(sample_val) > 50:
            sample_val = sample_val[:47] + "..."
        
        print(f"   {col:25} | {dtype:10} | {unique_vals:4} unique | {null_count:2} nulls | {sample_val}")
    
    print(f"\n🎯 Pattern Detection:")
    
    # Scenario patterns
    scenario_cols = [col for col in df.columns if 'scenario' in col.lower()]
    if scenario_cols:
        for col in scenario_cols:
            unique_scenarios = df[col].nunique()
            print(f"   Scenario column: {col} ({unique_scenarios} unique scenarios)")
    
    # Time patterns
    time_cols = [col for col in df.columns if 'time' in col.lower()]
    if time_cols:
        for col in time_cols:
            time_range = df[col].max() - df[col].min()
            print(f"   Time column: {col} (range: {time_range:,})")
    
    # Position patterns
    pos_cols = [col for col in df.columns if any(p in col.lower() for p in ['ego_x', 'ego_y', 'ego_heading'])]
    if pos_cols:
        print(f"   Position columns: {pos_cols}")
    
    # Horizon patterns
    horizon_cols = [col for col in df.columns if 'horizon' in col.lower()]
    if horizon_cols:
        print(f"   Horizon columns: {len(horizon_cols)} detected")
        
        # Analyze horizon structure
        for col in horizon_cols:
            if df[col].dtype == 'object':  # JSON strings
                try:
                    sample_json = json.loads(df[col].iloc[0])
                    if isinstance(sample_json, list):
                        print(f"     {col}: JSON array with {len(sample_json)} points")
                except:
                    print(f"     {col}: Non-JSON object column")
    
    print(f"\n🗺️  Recommended Mapping:")
    mapping = {}
    
    # Auto-detect mapping
    for col in df.columns:
        col_lower = col.lower()
        if 'timestamp' in col_lower:
            mapping['timestamp_col'] = col
        elif 'scenario_id' in col_lower:
            mapping['scenario_id_col'] = col
        elif col_lower == 'ego_x':
            mapping['x_col'] = col
        elif col_lower == 'ego_y':
            mapping['y_col'] = col
        elif col_lower == 'ego_heading':
            mapping['heading_col'] = col
        elif col_lower == 'ego_velocity_x':
            mapping['velocity_x_col'] = col
        elif col_lower == 'ego_velocity_y':
            mapping['velocity_y_col'] = col
        elif col_lower == 'ego_acceleration_x':
            mapping['acceleration_x_col'] = col
        elif col_lower == 'ego_acceleration_y':
            mapping['acceleration_y_col'] = col
        elif col_lower == 'tire_steering_angle':
            mapping['steering_angle_col'] = col
    
    # Check for horizon data
    if any('horizon_x' in col for col in df.columns):
        mapping['horizon_prefix'] = 'horizon'
        mapping['horizon_format'] = 'json_arrays'
    
    for key, value in mapping.items():
        print(f"   {key}: {value}")
    
    return mapping


def test_horizon_parsing(csv_file):
    """Test parsing of horizon JSON data."""
    print(f"\n🔄 Testing Horizon Data Parsing")
    print("=" * 40)
    
    df = pd.read_csv(csv_file)
    
    # Find horizon columns
    horizon_cols = [col for col in df.columns if 'horizon' in col.lower() and df[col].dtype == 'object']
    
    if not horizon_cols:
        print("   No horizon JSON columns found")
        return
    
    print(f"   Found {len(horizon_cols)} horizon columns")
    
    # Test parsing first row
    first_row = df.iloc[0]
    
    for col in horizon_cols[:3]:  # Test first 3 horizon columns
        try:
            horizon_data = json.loads(first_row[col])
            if isinstance(horizon_data, list):
                print(f"   {col}: {len(horizon_data)} points, range [{horizon_data[0]:.3f}, {horizon_data[-1]:.3f}]")
            else:
                print(f"   {col}: Non-list JSON data")
        except Exception as e:
            print(f"   {col}: Parse error - {e}")


def test_scenario_grouping(csv_file):
    """Test scenario grouping functionality."""
    print(f"\n📦 Testing Scenario Grouping")
    print("=" * 40)
    
    df = pd.read_csv(csv_file)
    
    if 'scenario_id' in df.columns:
        scenario_groups = df.groupby('scenario_id')
        
        print(f"   Total scenarios: {len(scenario_groups)}")
        
        for scenario_id, group in list(scenario_groups)[:3]:  # Show first 3
            duration = (group['timestamp_us'].max() - group['timestamp_us'].min()) / 1e6  # Convert to seconds
            print(f"   {scenario_id}: {len(group)} rows, {duration:.1f}s duration")
    else:
        print("   No scenario_id column found")


def simulate_trajectory_conversion(csv_file, num_samples=3):
    """Simulate the trajectory conversion process."""
    print(f"\n🎯 Simulating Trajectory Conversion")
    print("=" * 40)
    
    df = pd.read_csv(csv_file)
    
    print(f"   Converting {num_samples} sample rows to trajectory format...")
    
    for i in range(min(num_samples, len(df))):
        row = df.iloc[i]
        
        # Extract ego state
        ego_x = row['ego_x']
        ego_y = row['ego_y']
        ego_heading = row['ego_heading']
        timestamp = row['timestamp_us']
        
        print(f"   Row {i}: ego=({ego_x:.2f}, {ego_y:.2f}, {ego_heading:.3f}) at {timestamp}")
        
        # Test horizon parsing
        if 'horizon_x' in df.columns:
            try:
                horizon_x = json.loads(row['horizon_x'])
                horizon_y = json.loads(row['horizon_y'])
                print(f"     Horizon: {len(horizon_x)} points from ({horizon_x[0]:.2f}, {horizon_y[0]:.2f}) to ({horizon_x[-1]:.2f}, {horizon_y[-1]:.2f})")
            except Exception as e:
                print(f"     Horizon parse error: {e}")


def main():
    """Main test function."""
    csv_file = Path(__file__).parent / "test_planner_data.csv"
    
    if not csv_file.exists():
        print(f"❌ CSV file not found: {csv_file}")
        print("   Run generate_test_planner_data.py first")
        return
    
    print("🧪 CSV Analysis and Conversion Test")
    print("=" * 60)
    print(f"📁 Testing file: {csv_file}")
    
    # Test 1: Basic analysis
    mapping = analyze_csv_simple(str(csv_file))
    
    # Test 2: Horizon parsing
    test_horizon_parsing(str(csv_file))
    
    # Test 3: Scenario grouping
    test_scenario_grouping(str(csv_file))
    
    # Test 4: Trajectory conversion simulation
    simulate_trajectory_conversion(str(csv_file))
    
    print(f"\n✅ CSV Analysis Complete!")
    print(f"🚀 Ready for Phase 3 evaluation pipeline testing")


if __name__ == "__main__":
    main()