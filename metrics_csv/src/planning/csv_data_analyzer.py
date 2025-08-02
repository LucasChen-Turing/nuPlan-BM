#!/usr/bin/env python3
"""
CSV Data Structure Analyzer for nuPlan Integration

This module analyzes CSV data structure to understand the format and mapping requirements
for integrating real planner outputs into the nuPlan evaluation system.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import pandas as pd
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
import json
from pathlib import Path


@dataclass
class CSVColumnMapping:
    """Mapping configuration for CSV columns to nuPlan data structures."""
    
    # Time and identification
    timestamp_col: Optional[str] = None
    scenario_id_col: Optional[str] = None
    iteration_col: Optional[str] = None
    
    # Ego state columns
    x_col: Optional[str] = None
    y_col: Optional[str] = None
    heading_col: Optional[str] = None
    velocity_x_col: Optional[str] = None
    velocity_y_col: Optional[str] = None
    acceleration_x_col: Optional[str] = None
    acceleration_y_col: Optional[str] = None
    yaw_rate_col: Optional[str] = None
    steering_angle_col: Optional[str] = None
    
    # Horizon/trajectory columns (if present)
    horizon_prefix: Optional[str] = None
    horizon_length: Optional[int] = None
    horizon_dt: Optional[float] = None  # Time step for horizon
    
    # Additional metadata
    planner_mode_col: Optional[str] = None
    confidence_col: Optional[str] = None


class CSVDataAnalyzer:
    """
    Analyzes CSV data structure and provides mapping recommendations
    for integration with nuPlan evaluation system.
    """
    
    def __init__(self, csv_file_path: str):
        """
        Initialize analyzer with CSV file path.
        
        Args:
            csv_file_path: Path to the CSV file containing planner data
        """
        self.csv_file_path = Path(csv_file_path)
        self.df: Optional[pd.DataFrame] = None
        self.analysis_results: Dict[str, Any] = {}
        self.recommended_mapping: Optional[CSVColumnMapping] = None
        
    def load_csv(self, **kwargs) -> bool:
        """
        Load CSV file into pandas DataFrame.
        
        Args:
            **kwargs: Additional arguments for pd.read_csv()
            
        Returns:
            True if successful, False otherwise
        """
        try:
            self.df = pd.read_csv(self.csv_file_path, **kwargs)
            print(f"✓ Loaded CSV with {len(self.df)} rows and {len(self.df.columns)} columns")
            return True
        except Exception as e:
            print(f"✗ Failed to load CSV: {e}")
            return False
    
    def analyze_structure(self) -> Dict[str, Any]:
        """
        Perform comprehensive analysis of CSV structure.
        
        Returns:
            Dictionary containing analysis results
        """
        if self.df is None:
            raise ValueError("CSV not loaded. Call load_csv() first.")
        
        self.analysis_results = {
            "basic_info": self._analyze_basic_info(),
            "column_analysis": self._analyze_columns(),
            "data_patterns": self._analyze_data_patterns(),
            "temporal_analysis": self._analyze_temporal_structure(),
            "trajectory_analysis": self._analyze_trajectory_structure(),
            "quality_assessment": self._assess_data_quality()
        }
        
        return self.analysis_results
    
    def _analyze_basic_info(self) -> Dict[str, Any]:
        """Analyze basic dataset information."""
        return {
            "total_rows": len(self.df),
            "total_columns": len(self.df.columns),
            "column_names": list(self.df.columns),
            "data_types": dict(self.df.dtypes.astype(str)),
            "memory_usage_mb": self.df.memory_usage(deep=True).sum() / (1024*1024),
            "has_null_values": self.df.isnull().any().any(),
            "null_counts": dict(self.df.isnull().sum())
        }
    
    def _analyze_columns(self) -> Dict[str, Any]:
        """Analyze individual columns for pattern recognition."""
        column_analysis = {}
        
        for col in self.df.columns:
            col_data = self.df[col]
            
            analysis = {
                "dtype": str(col_data.dtype),
                "unique_values": col_data.nunique(),
                "null_count": col_data.isnull().sum(),
                "sample_values": col_data.dropna().head(5).tolist()
            }
            
            # Numeric column analysis
            if pd.api.types.is_numeric_dtype(col_data):
                analysis.update({
                    "min": float(col_data.min()) if not col_data.empty else None,
                    "max": float(col_data.max()) if not col_data.empty else None,
                    "mean": float(col_data.mean()) if not col_data.empty else None,
                    "std": float(col_data.std()) if not col_data.empty else None
                })
            
            # Pattern detection
            analysis["likely_types"] = self._detect_column_type(col, col_data)
            
            column_analysis[col] = analysis
        
        return column_analysis
    
    def _detect_column_type(self, col_name: str, col_data: pd.Series) -> List[str]:
        """Detect likely semantic meaning of a column."""
        likely_types = []
        col_lower = col_name.lower()
        
        # Time-related patterns
        if any(keyword in col_lower for keyword in ['time', 'timestamp', 't_']):
            likely_types.append("timestamp")
        
        # Position patterns
        if col_lower in ['x', 'pos_x', 'position_x', 'ego_x']:
            likely_types.append("position_x")
        elif col_lower in ['y', 'pos_y', 'position_y', 'ego_y']:
            likely_types.append("position_y")
        
        # Heading/orientation patterns
        if any(keyword in col_lower for keyword in ['heading', 'yaw', 'theta', 'orientation']):
            likely_types.append("heading")
        
        # Velocity patterns
        if any(keyword in col_lower for keyword in ['vel', 'speed', 'velocity']):
            if 'x' in col_lower:
                likely_types.append("velocity_x")
            elif 'y' in col_lower:
                likely_types.append("velocity_y")
            else:
                likely_types.append("velocity")
        
        # Acceleration patterns
        if any(keyword in col_lower for keyword in ['acc', 'acceleration']):
            if 'x' in col_lower:
                likely_types.append("acceleration_x")
            elif 'y' in col_lower:
                likely_types.append("acceleration_y")
            else:
                likely_types.append("acceleration")
        
        # Steering patterns
        if any(keyword in col_lower for keyword in ['steer', 'steering']):
            likely_types.append("steering_angle")
        
        # Scenario identification
        if any(keyword in col_lower for keyword in ['scenario', 'scene', 'id']):
            likely_types.append("scenario_id")
        
        # Horizon/trajectory patterns
        if any(keyword in col_lower for keyword in ['horizon', 'future', 'plan', 'traj']):
            likely_types.append("trajectory_data")
        
        return likely_types
    
    def _analyze_data_patterns(self) -> Dict[str, Any]:
        """Analyze data patterns and relationships."""
        patterns = {}
        
        # Check for sequential data
        numeric_cols = self.df.select_dtypes(include=[np.number]).columns
        for col in numeric_cols:
            if len(self.df[col].dropna()) > 1:
                diffs = self.df[col].diff().dropna()
                patterns[f"{col}_sequential"] = {
                    "appears_sequential": (diffs > 0).all() or (diffs < 0).all(),
                    "constant_step": diffs.std() < 0.01 * abs(diffs.mean()) if len(diffs) > 0 else False
                }
        
        return patterns
    
    def _analyze_temporal_structure(self) -> Dict[str, Any]:
        """Analyze temporal structure of the data."""
        temporal_analysis = {}
        
        # Look for time-like columns
        time_candidates = []
        for col in self.df.columns:
            col_analysis = self.analysis_results.get("column_analysis", {}).get(col, {})
            likely_types = col_analysis.get("likely_types", [])
            
            if "timestamp" in likely_types:
                time_candidates.append(col)
        
        temporal_analysis["time_candidates"] = time_candidates
        
        # Analyze each time candidate
        for col in time_candidates:
            try:
                time_series = pd.to_datetime(self.df[col])
                temporal_analysis[f"{col}_analysis"] = {
                    "successfully_parsed": True,
                    "time_range": {
                        "start": str(time_series.min()),
                        "end": str(time_series.max()),
                        "duration_seconds": (time_series.max() - time_series.min()).total_seconds()
                    },
                    "frequency_analysis": self._analyze_frequency(time_series)
                }
            except:
                temporal_analysis[f"{col}_analysis"] = {"successfully_parsed": False}
        
        return temporal_analysis
    
    def _analyze_frequency(self, time_series: pd.Series) -> Dict[str, Any]:
        """Analyze the frequency of time series data."""
        if len(time_series) < 2:
            return {"insufficient_data": True}
        
        time_diffs = time_series.diff().dropna()
        avg_dt = time_diffs.mean().total_seconds()
        std_dt = time_diffs.std().total_seconds()
        
        return {
            "average_dt_seconds": avg_dt,
            "std_dt_seconds": std_dt,
            "appears_regular": std_dt < 0.1 * avg_dt,
            "estimated_frequency_hz": 1.0 / avg_dt if avg_dt > 0 else None
        }
    
    def _analyze_trajectory_structure(self) -> Dict[str, Any]:
        """Analyze if data contains trajectory/horizon information."""
        trajectory_analysis = {}
        
        # Look for horizon-like column patterns
        horizon_patterns = []
        for col in self.df.columns:
            if any(pattern in col.lower() for pattern in ['horizon', 'future', 'plan_', 'traj_']):
                horizon_patterns.append(col)
        
        trajectory_analysis["horizon_candidates"] = horizon_patterns
        
        # Check for numbered sequence patterns (e.g., x_0, x_1, x_2...)
        sequence_groups = {}
        for col in self.df.columns:
            # Look for patterns like "prefix_number"
            parts = col.split('_')
            if len(parts) >= 2 and parts[-1].isdigit():
                prefix = '_'.join(parts[:-1])
                if prefix not in sequence_groups:
                    sequence_groups[prefix] = []
                sequence_groups[prefix].append(col)
        
        # Filter groups that look like trajectory sequences
        trajectory_sequences = {}
        for prefix, cols in sequence_groups.items():
            if len(cols) >= 3:  # At least 3 points for a trajectory
                trajectory_sequences[prefix] = {
                    "columns": sorted(cols),
                    "length": len(cols),
                    "indices": sorted([int(col.split('_')[-1]) for col in cols])
                }
        
        trajectory_analysis["trajectory_sequences"] = trajectory_sequences
        
        return trajectory_analysis
    
    def _assess_data_quality(self) -> Dict[str, Any]:
        """Assess data quality and completeness."""
        quality = {}
        
        # Overall completeness
        total_cells = self.df.size
        missing_cells = self.df.isnull().sum().sum()
        quality["completeness_ratio"] = (total_cells - missing_cells) / total_cells
        
        # Check for duplicate rows
        quality["duplicate_rows"] = len(self.df) - len(self.df.drop_duplicates())
        
        # Check for consistent data ranges
        numeric_cols = self.df.select_dtypes(include=[np.number]).columns
        outlier_analysis = {}
        for col in numeric_cols:
            col_data = self.df[col].dropna()
            if len(col_data) > 0:
                Q1 = col_data.quantile(0.25)
                Q3 = col_data.quantile(0.75)
                IQR = Q3 - Q1
                lower_bound = Q1 - 1.5 * IQR
                upper_bound = Q3 + 1.5 * IQR
                outliers = ((col_data < lower_bound) | (col_data > upper_bound)).sum()
                outlier_analysis[col] = {
                    "outlier_count": int(outliers),
                    "outlier_ratio": float(outliers / len(col_data))
                }
        
        quality["outlier_analysis"] = outlier_analysis
        
        return quality
    
    def generate_mapping_recommendations(self) -> CSVColumnMapping:
        """Generate recommended column mapping based on analysis."""
        if not self.analysis_results:
            raise ValueError("Run analyze_structure() first")
        
        mapping = CSVColumnMapping()
        column_analysis = self.analysis_results["column_analysis"]
        
        # Map columns based on detected types
        for col, analysis in column_analysis.items():
            likely_types = analysis.get("likely_types", [])
            
            if "timestamp" in likely_types:
                mapping.timestamp_col = col
            elif "scenario_id" in likely_types:
                mapping.scenario_id_col = col
            elif "position_x" in likely_types:
                mapping.x_col = col
            elif "position_y" in likely_types:
                mapping.y_col = col
            elif "heading" in likely_types:
                mapping.heading_col = col
            elif "velocity_x" in likely_types:
                mapping.velocity_x_col = col
            elif "velocity_y" in likely_types:
                mapping.velocity_y_col = col
            elif "acceleration_x" in likely_types:
                mapping.acceleration_x_col = col
            elif "acceleration_y" in likely_types:
                mapping.acceleration_y_col = col
            elif "steering_angle" in likely_types:
                mapping.steering_angle_col = col
        
        # Analyze trajectory sequences
        trajectory_sequences = self.analysis_results["trajectory_analysis"]["trajectory_sequences"]
        if trajectory_sequences:
            # Pick the first/largest sequence as horizon data
            largest_seq = max(trajectory_sequences.items(), key=lambda x: x[1]["length"])
            mapping.horizon_prefix = largest_seq[0]
            mapping.horizon_length = largest_seq[1]["length"]
        
        self.recommended_mapping = mapping
        return mapping
    
    def export_analysis(self, output_path: Optional[str] = None) -> str:
        """Export analysis results to JSON file."""
        if not self.analysis_results:
            raise ValueError("No analysis results to export")
        
        if output_path is None:
            output_path = self.csv_file_path.parent / f"{self.csv_file_path.stem}_analysis.json"
        
        # Convert numpy types to Python types for JSON serialization
        def convert_types(obj):
            if isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, dict):
                return {key: convert_types(value) for key, value in obj.items()}
            elif isinstance(obj, list):
                return [convert_types(item) for item in obj]
            return obj
        
        export_data = {
            "csv_file": str(self.csv_file_path),
            "analysis_timestamp": pd.Timestamp.now().isoformat(),
            "analysis_results": convert_types(self.analysis_results),
            "recommended_mapping": convert_types(self.recommended_mapping.__dict__) if self.recommended_mapping else None
        }
        
        with open(output_path, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"✓ Analysis exported to: {output_path}")
        return str(output_path)
    
    def print_summary(self):
        """Print a human-readable summary of the analysis."""
        if not self.analysis_results:
            print("No analysis results available. Run analyze_structure() first.")
            return
        
        basic_info = self.analysis_results["basic_info"]
        temporal_analysis = self.analysis_results["temporal_analysis"]
        trajectory_analysis = self.analysis_results["trajectory_analysis"]
        quality = self.analysis_results["quality_assessment"]
        
        print("=" * 60)
        print("CSV DATA STRUCTURE ANALYSIS SUMMARY")
        print("=" * 60)
        
        print(f"\n📊 BASIC INFO:")
        print(f"   Rows: {basic_info['total_rows']:,}")
        print(f"   Columns: {basic_info['total_columns']}")
        print(f"   Memory: {basic_info['memory_usage_mb']:.1f} MB")
        print(f"   Has null values: {basic_info['has_null_values']}")
        
        print(f"\n⏰ TEMPORAL STRUCTURE:")
        time_candidates = temporal_analysis.get("time_candidates", [])
        print(f"   Time columns found: {len(time_candidates)}")
        for col in time_candidates:
            analysis = temporal_analysis.get(f"{col}_analysis", {})
            if analysis.get("successfully_parsed"):
                freq = analysis["frequency_analysis"].get("estimated_frequency_hz", "Unknown")
                print(f"   - {col}: {freq:.1f} Hz" if isinstance(freq, (int, float)) else f"   - {col}: {freq}")
        
        print(f"\n🎯 TRAJECTORY STRUCTURE:")
        trajectory_sequences = trajectory_analysis.get("trajectory_sequences", {})
        print(f"   Trajectory sequences found: {len(trajectory_sequences)}")
        for prefix, info in trajectory_sequences.items():
            print(f"   - {prefix}: {info['length']} points")
        
        print(f"\n✅ DATA QUALITY:")
        print(f"   Completeness: {quality['completeness_ratio']:.1%}")
        print(f"   Duplicate rows: {quality['duplicate_rows']}")
        
        if self.recommended_mapping:
            print(f"\n🗺️  RECOMMENDED MAPPING:")
            mapping = self.recommended_mapping
            for field_name, value in mapping.__dict__.items():
                if value is not None:
                    print(f"   {field_name}: {value}")
        
        print("=" * 60)


def main():
    """Main function for testing the analyzer."""
    print("CSV Data Structure Analyzer - Test Mode")
    print("Usage: python csv_data_analyzer.py <csv_file_path>")
    
    # Example usage (for testing)
    # analyzer = CSVDataAnalyzer("path/to/your/csv/file.csv")
    # analyzer.load_csv()
    # analyzer.analyze_structure()
    # analyzer.generate_mapping_recommendations()
    # analyzer.print_summary()
    # analyzer.export_analysis()


if __name__ == "__main__":
    main()