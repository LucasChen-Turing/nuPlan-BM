#!/usr/bin/env python3
"""
CSV LogFuture Planner

A planner that reads trajectory data from CSV files instead of database scenarios.
This extends nuPlan's LogFuturePlanner to work with CSV trajectory data for offline evaluation.
"""

import itertools
import logging
import pandas as pd
from pathlib import Path
from typing import List, Optional, Type

from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.car_footprint import CarFootprint
from nuplan.common.actor_state.dynamic_car_state import DynamicCarState
from nuplan.common.actor_state.state_representation import StateSE2, StateVector2D, TimePoint
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters
from nuplan.planning.simulation.observation.observation_type import DetectionsTracks, Observation
from nuplan.planning.simulation.planner.abstract_planner import AbstractPlanner, PlannerInitialization, PlannerInput
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory
from nuplan.planning.simulation.trajectory.interpolated_trajectory import InterpolatedTrajectory

logger = logging.getLogger(__name__)


class CSVLogFuturePlanner(AbstractPlanner):
    """
    A planner that reads future trajectory from CSV files instead of scenario database.
    
    This mimics nuPlan's LogFuturePlanner but uses CSV data as the trajectory source.
    Perfect for offline evaluation of pre-computed trajectories.
    """

    def __init__(
        self, 
        csv_file_path: str,
        num_poses: int = 16, 
        future_time_horizon: float = 8.0,
        scenario_id_column: str = "scenario_id",
        trajectory_column_prefix: str = "ego_"
    ):
        """
        Constructor for CSVLogFuturePlanner.
        
        :param csv_file_path: Path to CSV file containing trajectory data
        :param num_poses: Number of poses to include in trajectory
        :param future_time_horizon: [s] Time horizon for trajectory
        :param scenario_id_column: Column name for scenario identification
        :param trajectory_column_prefix: Prefix for trajectory columns
        """
        super().__init__()
        
        self.csv_file_path = Path(csv_file_path)
        self._num_poses = num_poses
        self._future_time_horizon = future_time_horizon
        self.scenario_id_column = scenario_id_column
        self.trajectory_column_prefix = trajectory_column_prefix
        
        # Vehicle parameters
        self.vehicle_parameters = get_pacifica_parameters()
        
        # Load CSV data
        self._load_trajectory_data()
        
        # Current state
        self._current_scenario_id: Optional[str] = None
        self._current_trajectory_data: Optional[pd.DataFrame] = None
        self._trajectory: Optional[AbstractTrajectory] = None

    def _load_trajectory_data(self) -> None:
        """Load trajectory data from CSV file."""
        if not self.csv_file_path.exists():
            raise FileNotFoundError(f"CSV trajectory file not found: {self.csv_file_path}")
        
        try:
            self.trajectory_df = pd.read_csv(self.csv_file_path)
            logger.info(f"Loaded trajectory data: {len(self.trajectory_df)} rows")
            
            # Validate required columns
            required_columns = [
                f"{self.trajectory_column_prefix}x",
                f"{self.trajectory_column_prefix}y", 
                f"{self.trajectory_column_prefix}heading",
                f"{self.trajectory_column_prefix}velocity_x",
                f"{self.trajectory_column_prefix}velocity_y",
                "timestamp_us",
                "iteration",
                self.scenario_id_column
            ]
            
            missing_columns = [col for col in required_columns if col not in self.trajectory_df.columns]
            if missing_columns:
                raise ValueError(f"Missing required columns in CSV: {missing_columns}")
            
            # Group trajectories by scenario ID
            self.scenario_trajectories = {}
            for scenario_id, group in self.trajectory_df.groupby(self.scenario_id_column):
                # Sort by iteration to ensure correct order
                sorted_group = group.sort_values('iteration').reset_index(drop=True)
                self.scenario_trajectories[scenario_id] = sorted_group
                
            logger.info(f"Loaded {len(self.scenario_trajectories)} unique scenarios")
            
        except Exception as e:
            raise RuntimeError(f"Failed to load CSV trajectory data: {e}")

    def set_current_scenario(self, scenario_id: str) -> None:
        """
        Set the current scenario to extract trajectory from.
        
        :param scenario_id: ID of the scenario to use
        """
        if scenario_id not in self.scenario_trajectories:
            available_scenarios = list(self.scenario_trajectories.keys())
            raise ValueError(f"Scenario '{scenario_id}' not found. Available: {available_scenarios}")
        
        self._current_scenario_id = scenario_id
        self._current_trajectory_data = self.scenario_trajectories[scenario_id]
        logger.info(f"Set current scenario: {scenario_id} ({len(self._current_trajectory_data)} points)")

    def initialize(self, initialization: PlannerInitialization) -> None:
        """Inherited, see superclass."""
        # Use first available scenario if none set
        if self._current_scenario_id is None and self.scenario_trajectories:
            first_scenario_id = list(self.scenario_trajectories.keys())[0]
            self.set_current_scenario(first_scenario_id)

    def name(self) -> str:
        """Inherited, see superclass."""
        return self.__class__.__name__

    def observation_type(self) -> Type[Observation]:
        """Inherited, see superclass."""
        return DetectionsTracks  # type: ignore

    def compute_planner_trajectory(self, current_input: PlannerInput) -> AbstractTrajectory:
        """
        Compute trajectory by reading from CSV data.
        
        This mimics LogFuturePlanner's behavior but reads from CSV instead of scenario database.
        """
        if self._current_trajectory_data is None:
            raise RuntimeError("No trajectory data loaded. Call set_current_scenario() first.")
        
        current_iteration = current_input.iteration.index
        current_time = current_input.iteration.time_point
        
        # Find corresponding CSV row based on iteration
        csv_start_idx = None
        for idx, row in self._current_trajectory_data.iterrows():
            if int(row['iteration']) >= current_iteration:
                csv_start_idx = idx
                break
        
        if csv_start_idx is None:
            # Use previous trajectory if we've exceeded CSV data
            if self._trajectory is not None:
                logger.warning("Cannot retrieve future trajectory from CSV. Using previous computed trajectory.")
                return self._trajectory
            else:
                raise RuntimeError("Future trajectory cannot be retrieved from CSV data!")
        
        # Extract future trajectory points
        end_idx = min(csv_start_idx + self._num_poses, len(self._current_trajectory_data))
        future_data = self._current_trajectory_data.iloc[csv_start_idx:end_idx]
        
        # Ensure we have enough points
        if len(future_data) < 2:
            if self._trajectory is not None:
                logger.warning("Insufficient future data in CSV. Using previous computed trajectory.")
                return self._trajectory
            else:
                # Use last available points
                future_data = self._current_trajectory_data.iloc[-2:]
        
        # Convert CSV data to EgoState trajectory
        ego_states = self._csv_to_ego_states(future_data, current_time)
        
        # Create trajectory (mimicking LogFuturePlanner's approach)
        self._trajectory = InterpolatedTrajectory(ego_states)
        
        return self._trajectory

    def _csv_to_ego_states(self, csv_data: pd.DataFrame, base_time: TimePoint) -> List[EgoState]:
        """
        Convert CSV data to list of EgoState objects.
        
        :param csv_data: DataFrame with trajectory data
        :param base_time: Base time for trajectory
        :return: List of EgoState objects
        """
        ego_states = []
        
        for idx, (_, row) in enumerate(csv_data.iterrows()):
            # Extract pose information
            x = float(row[f"{self.trajectory_column_prefix}x"])
            y = float(row[f"{self.trajectory_column_prefix}y"])
            heading = float(row[f"{self.trajectory_column_prefix}heading"])
            
            # Extract velocity information
            vx = float(row[f"{self.trajectory_column_prefix}velocity_x"])
            vy = float(row[f"{self.trajectory_column_prefix}velocity_y"])
            
            # Extract optional fields with defaults
            ax = float(row.get(f"{self.trajectory_column_prefix}acceleration_x", 0.0))
            ay = float(row.get(f"{self.trajectory_column_prefix}acceleration_y", 0.0))
            angular_velocity = float(row.get(f"{self.trajectory_column_prefix}angular_velocity", 0.0))
            angular_acceleration = float(row.get(f"{self.trajectory_column_prefix}angular_acceleration", 0.0))
            tire_steering_angle = float(row.get("tire_steering_angle", 0.0))
            
            # Create timestamp - use CSV timestamp or calculate from base time
            if 'timestamp_us' in row and pd.notna(row['timestamp_us']):
                timestamp_us = int(row['timestamp_us'])
            else:
                # Calculate based on iteration and typical nuPlan sampling (4Hz = 250ms)
                iteration = int(row['iteration'])
                timestamp_us = base_time.time_us + iteration * 250000
            
            time_point = TimePoint(timestamp_us)
            
            # Create pose
            pose = StateSE2(x, y, heading)
            
            # Create car footprint
            car_footprint = CarFootprint.build_from_center(
                center=pose,
                vehicle_parameters=self.vehicle_parameters
            )
            
            # Create dynamic state
            rear_axle_velocity = StateVector2D(vx, vy)
            rear_axle_acceleration = StateVector2D(ax, ay)
            
            dynamic_state = DynamicCarState.build_from_rear_axle(
                rear_axle_to_center_dist=self.vehicle_parameters.rear_axle_to_center,
                rear_axle_velocity_2d=rear_axle_velocity,
                rear_axle_acceleration_2d=rear_axle_acceleration,
                angular_velocity=angular_velocity,
                angular_acceleration=angular_acceleration
            )
            
            # Create EgoState
            ego_state = EgoState(
                car_footprint=car_footprint,
                dynamic_car_state=dynamic_state,
                tire_steering_angle=tire_steering_angle,
                is_in_auto_mode=True,
                time_point=time_point
            )
            
            ego_states.append(ego_state)
        
        return ego_states

    def get_available_scenarios(self) -> List[str]:
        """Get list of available scenario IDs."""
        return list(self.scenario_trajectories.keys())


# Factory function for configuration integration
def create_csv_log_future_planner(
    csv_file_path: str,
    num_poses: int = 16,
    future_time_horizon: float = 8.0,
    **kwargs
) -> CSVLogFuturePlanner:
    """
    Factory function to create CSVLogFuturePlanner instance.
    
    :param csv_file_path: Path to CSV trajectory file
    :param num_poses: Number of poses in trajectory
    :param future_time_horizon: Time horizon in seconds
    :param kwargs: Additional parameters
    :return: Configured CSVLogFuturePlanner instance
    """
    return CSVLogFuturePlanner(
        csv_file_path=csv_file_path,
        num_poses=num_poses,
        future_time_horizon=future_time_horizon,
        **kwargs
    )