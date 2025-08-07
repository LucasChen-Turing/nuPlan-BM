#!/usr/bin/env python3
"""
Scenario Matching System for CSV-Database Alignment

This module provides mechanisms to match CSV planner data with corresponding
expert trajectories from nuPlan database scenarios.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import pandas as pd
import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass
from pathlib import Path
import json

from nuplan.common.actor_state.state_representation import TimePoint
from nuplan.planning.scenario_builder.nuplan_db.nuplan_scenario import NuPlanScenario

from .csv_data_analyzer import CSVColumnMapping


@dataclass
class ScenarioMatch:
    """Represents a match between CSV data and database scenario."""
    
    scenario_id: str
    scenario_name: str
    csv_rows: List[int]  # Row indices in CSV that match this scenario
    expert_trajectory_length: int
    time_alignment_offset: float  # Time offset to align CSV with expert
    matching_confidence: float  # 0.0 to 1.0 confidence score
    matching_method: str  # How the match was determined
    metadata: Dict[str, Any]  # Additional matching information


@dataclass
class MatchingConfig:
    """Configuration for scenario matching algorithms."""
    
    # Temporal matching
    max_time_difference: float = 2.0  # Max time difference for temporal alignment (seconds)
    time_tolerance: float = 0.5  # Tolerance for time matching (seconds)
    
    # Spatial matching
    max_spatial_distance: float = 50.0  # Max spatial distance for position matching (meters)
    spatial_tolerance: float = 5.0  # Tolerance for spatial matching (meters)
    
    # Trajectory matching
    min_trajectory_overlap: float = 0.7  # Minimum overlap ratio for trajectory matching
    trajectory_sample_rate: int = 10  # Sample every N points for trajectory comparison
    
    # ID-based matching
    use_scenario_id_matching: bool = True
    scenario_id_prefix_match: bool = True  # Allow prefix matching for scenario IDs
    
    # Quality thresholds
    min_confidence_threshold: float = 0.6  # Minimum confidence for accepting a match
    max_matches_per_scenario: int = 1  # Maximum CSV sequences per database scenario


class ScenarioMatcher:
    """
    Matches CSV planner data with nuPlan database scenarios.
    
    Supports multiple matching strategies:
    1. Direct scenario ID matching
    2. Temporal alignment matching
    3. Spatial trajectory matching
    4. Combined multi-criteria matching
    """
    
    def __init__(
        self, 
        csv_data: pd.DataFrame,
        column_mapping: CSVColumnMapping,
        config: Optional[MatchingConfig] = None
    ):
        """
        Initialize scenario matcher.
        
        Args:
            csv_data: DataFrame containing CSV planner data
            column_mapping: Mapping of CSV columns to data fields
            config: Matching configuration (uses defaults if None)
        """
        self.csv_data = csv_data
        self.mapping = column_mapping
        self.config = config or MatchingConfig()
        
        # Processed data
        self.csv_scenarios: Dict[str, List[int]] = {}  # scenario_id -> row indices
        self.matches: List[ScenarioMatch] = []
        
        # Prepare CSV data for matching
        self._prepare_csv_data()
    
    def _prepare_csv_data(self):
        """Prepare CSV data for matching operations."""
        print("Preparing CSV data for scenario matching...")
        
        # Group CSV rows by scenario if scenario column exists
        if self.mapping.scenario_id_col and self.mapping.scenario_id_col in self.csv_data.columns:
            scenario_groups = self.csv_data.groupby(self.mapping.scenario_id_col)
            for scenario_id, group in scenario_groups:
                self.csv_scenarios[str(scenario_id)] = group.index.tolist()
            
            print(f"   Found {len(self.csv_scenarios)} unique scenarios in CSV")
        else:
            # Treat entire CSV as one sequence
            self.csv_scenarios["unknown_scenario"] = list(self.csv_data.index)
            print("   No scenario ID column - treating CSV as single sequence")
        
        # Validate required columns
        required_cols = [self.mapping.x_col, self.mapping.y_col]
        missing_cols = [col for col in required_cols if col not in self.csv_data.columns]
        
        if missing_cols:
            raise ValueError(f"Missing required columns for matching: {missing_cols}")
    
    def match_by_scenario_id(
        self, 
        available_scenarios: List[NuPlanScenario]
    ) -> List[ScenarioMatch]:
        """
        Match CSV data to scenarios based on scenario ID.
        
        Args:
            available_scenarios: List of available database scenarios
            
        Returns:
            List of scenario matches
        """
        print("Attempting scenario ID-based matching...")
        
        matches = []
        scenario_lookup = {scenario.scenario_name: scenario for scenario in available_scenarios}
        
        for csv_scenario_id, csv_rows in self.csv_scenarios.items():
            best_match = None
            best_confidence = 0.0
            
            # Try exact match first
            if csv_scenario_id in scenario_lookup:
                scenario = scenario_lookup[csv_scenario_id]
                expert_trajectory = list(scenario.get_expert_ego_trajectory())
                
                match = ScenarioMatch(
                    scenario_id=csv_scenario_id,
                    scenario_name=scenario.scenario_name,
                    csv_rows=csv_rows,
                    expert_trajectory_length=len(expert_trajectory),
                    time_alignment_offset=0.0,
                    matching_confidence=1.0,
                    matching_method="exact_id_match",
                    metadata={"scenario_type": scenario.scenario_type}
                )
                matches.append(match)
                continue
            
            # Try prefix matching if enabled
            if self.config.scenario_id_prefix_match:
                for db_scenario_name, scenario in scenario_lookup.items():
                    # Check if CSV scenario ID is a prefix of database scenario name
                    if (db_scenario_name.startswith(csv_scenario_id) or 
                        csv_scenario_id.startswith(db_scenario_name)):
                        
                        confidence = len(csv_scenario_id) / max(len(csv_scenario_id), len(db_scenario_name))
                        
                        if confidence > best_confidence:
                            expert_trajectory = list(scenario.get_expert_ego_trajectory())
                            
                            best_match = ScenarioMatch(
                                scenario_id=csv_scenario_id,
                                scenario_name=scenario.scenario_name,
                                csv_rows=csv_rows,
                                expert_trajectory_length=len(expert_trajectory),
                                time_alignment_offset=0.0,
                                matching_confidence=confidence,
                                matching_method="prefix_id_match",
                                metadata={"scenario_type": scenario.scenario_type}
                            )
                            best_confidence = confidence
            
            if best_match and best_confidence >= self.config.min_confidence_threshold:
                matches.append(best_match)
        
        print(f"   Found {len(matches)} ID-based matches")
        return matches
    
    def match_by_temporal_alignment(
        self, 
        available_scenarios: List[NuPlanScenario]
    ) -> List[ScenarioMatch]:
        """
        Match CSV data to scenarios based on temporal alignment.
        
        Args:
            available_scenarios: List of available database scenarios
            
        Returns:
            List of scenario matches
        """
        print("Attempting temporal alignment matching...")
        
        if not self.mapping.timestamp_col or self.mapping.timestamp_col not in self.csv_data.columns:
            print("   No timestamp column available - skipping temporal matching")
            return []
        
        matches = []
        
        for csv_scenario_id, csv_rows in self.csv_scenarios.items():
            csv_subset = self.csv_data.iloc[csv_rows]
            csv_timestamps = csv_subset[self.mapping.timestamp_col].values
            csv_duration = csv_timestamps[-1] - csv_timestamps[0]
            
            best_match = None
            best_confidence = 0.0
            
            for scenario in available_scenarios:
                expert_trajectory = list(scenario.get_expert_ego_trajectory())
                if len(expert_trajectory) < 2:
                    continue
                
                expert_duration = expert_trajectory[-1].time_point.time_s - expert_trajectory[0].time_point.time_s
                
                # Check duration compatibility
                duration_ratio = min(csv_duration, expert_duration) / max(csv_duration, expert_duration)
                
                if duration_ratio < 0.5:  # Durations too different
                    continue
                
                # Calculate temporal alignment confidence
                confidence = duration_ratio * 0.8  # Base confidence from duration similarity
                
                # Add bonus for similar trajectory length
                length_ratio = min(len(csv_rows), len(expert_trajectory)) / max(len(csv_rows), len(expert_trajectory))
                confidence += length_ratio * 0.2
                
                if confidence > best_confidence:
                    # Calculate time alignment offset
                    csv_start_time = csv_timestamps[0]
                    expert_start_time = expert_trajectory[0].time_point.time_s
                    time_offset = expert_start_time - csv_start_time
                    
                    best_match = ScenarioMatch(
                        scenario_id=csv_scenario_id,
                        scenario_name=scenario.scenario_name,
                        csv_rows=csv_rows,
                        expert_trajectory_length=len(expert_trajectory),
                        time_alignment_offset=time_offset,
                        matching_confidence=confidence,
                        matching_method="temporal_alignment",
                        metadata={
                            "scenario_type": scenario.scenario_type,
                            "csv_duration": csv_duration,
                            "expert_duration": expert_duration,
                            "duration_ratio": duration_ratio
                        }
                    )
                    best_confidence = confidence
            
            if best_match and best_confidence >= self.config.min_confidence_threshold:
                matches.append(best_match)
        
        print(f"   Found {len(matches)} temporal matches")
        return matches
    
    def match_by_spatial_trajectory(
        self, 
        available_scenarios: List[NuPlanScenario]
    ) -> List[ScenarioMatch]:
        """
        Match CSV data to scenarios based on spatial trajectory similarity.
        
        Args:
            available_scenarios: List of available database scenarios
            
        Returns:
            List of scenario matches
        """
        print("Attempting spatial trajectory matching...")
        
        matches = []
        
        for csv_scenario_id, csv_rows in self.csv_scenarios.items():
            csv_subset = self.csv_data.iloc[csv_rows]
            
            # Extract CSV trajectory
            csv_x = csv_subset[self.mapping.x_col].values
            csv_y = csv_subset[self.mapping.y_col].values
            
            if len(csv_x) < 3:  # Need minimum points for trajectory comparison
                continue
            
            best_match = None
            best_confidence = 0.0
            
            for scenario in available_scenarios:
                expert_trajectory = list(scenario.get_expert_ego_trajectory())
                if len(expert_trajectory) < 3:
                    continue
                
                # Extract expert trajectory
                expert_x = np.array([state.rear_axle.x for state in expert_trajectory])
                expert_y = np.array([state.rear_axle.y for state in expert_trajectory])
                
                # Calculate spatial similarity
                confidence = self._calculate_trajectory_similarity(
                    csv_x, csv_y, expert_x, expert_y
                )
                
                if confidence > best_confidence:
                    best_match = ScenarioMatch(
                        scenario_id=csv_scenario_id,
                        scenario_name=scenario.scenario_name,
                        csv_rows=csv_rows,
                        expert_trajectory_length=len(expert_trajectory),
                        time_alignment_offset=0.0,
                        matching_confidence=confidence,
                        matching_method="spatial_trajectory",
                        metadata={
                            "scenario_type": scenario.scenario_type,
                            "csv_trajectory_length": len(csv_x),
                            "expert_trajectory_length": len(expert_x)
                        }
                    )
                    best_confidence = confidence
            
            if best_match and best_confidence >= self.config.min_confidence_threshold:
                matches.append(best_match)
        
        print(f"   Found {len(matches)} spatial matches")
        return matches
    
    def _calculate_trajectory_similarity(
        self, 
        csv_x: np.ndarray, 
        csv_y: np.ndarray,
        expert_x: np.ndarray, 
        expert_y: np.ndarray
    ) -> float:
        """
        Calculate similarity between two trajectories.
        
        Args:
            csv_x, csv_y: CSV trajectory coordinates
            expert_x, expert_y: Expert trajectory coordinates
            
        Returns:
            Similarity score between 0.0 and 1.0
        """
        # Sample trajectories at regular intervals for comparison
        sample_rate = self.config.trajectory_sample_rate
        
        # Normalize trajectory lengths by sampling
        csv_length = len(csv_x)
        expert_length = len(expert_x)
        
        # Sample points for comparison
        csv_indices = np.linspace(0, csv_length - 1, min(csv_length, sample_rate), dtype=int)
        expert_indices = np.linspace(0, expert_length - 1, min(expert_length, sample_rate), dtype=int)
        
        csv_sample_x = csv_x[csv_indices]
        csv_sample_y = csv_y[csv_indices]
        expert_sample_x = expert_x[expert_indices]
        expert_sample_y = expert_y[expert_indices]
        
        # Align trajectories by translating to same starting point
        csv_sample_x = csv_sample_x - csv_sample_x[0]
        csv_sample_y = csv_sample_y - csv_sample_y[0]
        expert_sample_x = expert_sample_x - expert_sample_x[0]
        expert_sample_y = expert_sample_y - expert_sample_y[0]
        
        # Calculate distances between corresponding points
        min_points = min(len(csv_sample_x), len(expert_sample_x))
        
        distances = []
        for i in range(min_points):
            # Find closest point in expert trajectory to CSV point i
            csv_point = np.array([csv_sample_x[i], csv_sample_y[i]])
            expert_points = np.column_stack([expert_sample_x, expert_sample_y])
            
            point_distances = np.linalg.norm(expert_points - csv_point, axis=1)
            min_distance = np.min(point_distances)
            distances.append(min_distance)
        
        # Calculate similarity score
        avg_distance = np.mean(distances)
        max_distance = self.config.max_spatial_distance
        
        # Convert distance to similarity (0 distance = 1.0 similarity)
        similarity = max(0.0, 1.0 - (avg_distance / max_distance))
        
        return similarity
    
    def find_best_matches(
        self, 
        available_scenarios: List[NuPlanScenario]
    ) -> List[ScenarioMatch]:
        """
        Find the best matches using all available matching strategies.
        
        Args:
            available_scenarios: List of available database scenarios
            
        Returns:
            List of best scenario matches
        """
        print("Finding best scenario matches using all strategies...")
        
        all_matches = []
        
        # Try ID-based matching first (highest confidence)
        if self.config.use_scenario_id_matching:
            id_matches = self.match_by_scenario_id(available_scenarios)
            all_matches.extend(id_matches)
        
        # Try temporal matching for unmatched scenarios
        temporal_matches = self.match_by_temporal_alignment(available_scenarios)
        all_matches.extend(temporal_matches)
        
        # Try spatial matching for still unmatched scenarios
        spatial_matches = self.match_by_spatial_trajectory(available_scenarios)
        all_matches.extend(spatial_matches)
        
        # Remove duplicates and select best matches
        best_matches = self._select_best_matches(all_matches)
        
        print(f"Selected {len(best_matches)} best matches")
        return best_matches
    
    def _select_best_matches(self, all_matches: List[ScenarioMatch]) -> List[ScenarioMatch]:
        """
        Select the best matches from all candidates, avoiding duplicates.
        
        Args:
            all_matches: All potential matches
            
        Returns:
            Best unique matches
        """
        # Group matches by CSV scenario ID
        matches_by_csv_scenario: Dict[str, List[ScenarioMatch]] = {}
        for match in all_matches:
            if match.scenario_id not in matches_by_csv_scenario:
                matches_by_csv_scenario[match.scenario_id] = []
            matches_by_csv_scenario[match.scenario_id].append(match)
        
        # Select best match for each CSV scenario
        best_matches = []
        used_db_scenarios = set()
        
        for csv_scenario_id, matches in matches_by_csv_scenario.items():
            # Sort by confidence, then by matching method priority
            method_priority = {"exact_id_match": 3, "prefix_id_match": 2, "temporal_alignment": 1, "spatial_trajectory": 0}
            
            matches.sort(
                key=lambda m: (m.matching_confidence, method_priority.get(m.matching_method, 0)),
                reverse=True
            )
            
            # Select best match that doesn't conflict with already used scenarios
            for match in matches:
                if (match.scenario_name not in used_db_scenarios and 
                    match.matching_confidence >= self.config.min_confidence_threshold):
                    
                    best_matches.append(match)
                    used_db_scenarios.add(match.scenario_name)
                    break
        
        return best_matches
    
    def export_matches(self, output_path: str):
        """
        Export scenario matches to JSON file.
        
        Args:
            output_path: Path to save matches
        """
        export_data = []
        
        for match in self.matches:
            export_data.append({
                "scenario_id": match.scenario_id,
                "scenario_name": match.scenario_name,
                "csv_rows": match.csv_rows,
                "expert_trajectory_length": match.expert_trajectory_length,
                "time_alignment_offset": match.time_alignment_offset,
                "matching_confidence": match.matching_confidence,
                "matching_method": match.matching_method,
                "metadata": match.metadata
            })
        
        with open(output_path, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"Exported {len(export_data)} matches to: {output_path}")


def main():
    """Main function for testing scenario matching."""
    print("Scenario Matcher - Test Mode")
    print("This module provides CSV-to-database scenario matching functionality.")


if __name__ == "__main__":
    main()