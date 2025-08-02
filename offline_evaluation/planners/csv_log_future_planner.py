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

    # Inherited property, see superclass.
    requires_scenario: bool = True

    def __init__(
        self, 
        csv_file_path: str,
        scenario=None,  # Accept scenario parameter like LogFuturePlanner
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
        
        # Store nuPlan scenario for timestamp alignment (passed at construction)
        self._scenario = scenario  # This comes from the constructor parameter
        self._scenario_start_time: Optional[int] = None
        
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
        
        # Get scenario start time for timestamp alignment
        self._scenario_start_time = self._get_scenario_start_time()

    def initialize(self, initialization: PlannerInitialization) -> None:
        """Inherited, see superclass."""
        # Scenario is already set during construction
        if self._scenario is None:
            raise RuntimeError("No scenario provided during planner construction")
        
        # Try to match scenario with CSV data using nuplan_token
        scenario_token = self._scenario.token
        if isinstance(scenario_token, bytes):
            scenario_token = scenario_token.hex()
        
        matching_scenario_id = None
        
        # Look for matching token in CSV data
        if 'nuplan_token' in self.trajectory_df.columns:
            matching_rows = self.trajectory_df[self.trajectory_df['nuplan_token'] == scenario_token]
            if not matching_rows.empty:
                matching_scenario_id = matching_rows.iloc[0][self.scenario_id_column]
                logger.info(f"Found matching scenario for token {scenario_token}: {matching_scenario_id}")
        
        # Set current scenario based on match or use first available
        if matching_scenario_id and matching_scenario_id in self.scenario_trajectories:
            self.set_current_scenario(matching_scenario_id)
        elif self.scenario_trajectories:
            first_scenario_id = list(self.scenario_trajectories.keys())[0]
            logger.warning(f"No matching scenario found for token {scenario_token}, using first available: {first_scenario_id}")
            self.set_current_scenario(first_scenario_id)
        else:
            raise RuntimeError("No trajectory data available in CSV file")

    def name(self) -> str:
        """Inherited, see superclass."""
        return self.__class__.__name__

    def observation_type(self) -> Type[Observation]:
        """Inherited, see superclass."""
        return DetectionsTracks  # type: ignore

    def compute_planner_trajectory(self, current_input: PlannerInput) -> AbstractTrajectory:
        """Inherited, see superclass."""
        # Use the exact same structure as LogFuturePlanner but with CSV data
        current_state = self._scenario.get_ego_state_at_iteration(current_input.iteration.index)
        try:
            # Get CSV-based future trajectory that mimics scenario.get_ego_future_trajectory behavior
            states = self._get_csv_future_trajectory(
                current_input.iteration.index, self._future_time_horizon, self._num_poses
            )
            # Use itertools.chain exactly like LogFuturePlanner 
            import itertools
            self._trajectory = InterpolatedTrajectory(list(itertools.chain([current_state], states)))
        except AssertionError:
            logger.warning("Cannot retrieve CSV future trajectory. Using previous computed trajectory.")
            if self._trajectory is None:
                raise RuntimeError("CSV future trajectory cannot be retrieved!")

        return self._trajectory
    
    def _get_csv_future_trajectory(self, iteration_index: int, future_time_horizon: float, num_poses: int) -> List[EgoState]:
        """
        Get future trajectory from CSV data, mimicking scenario.get_ego_future_trajectory() behavior exactly.
        
        :param iteration_index: Current iteration index
        :param future_time_horizon: Time horizon in seconds
        :param num_poses: Number of poses to return
        :return: List of future ego states
        """
        if self._current_trajectory_data is None:
            raise RuntimeError("No trajectory data loaded. Call set_current_scenario() first.")
        
        # Find matching CSV data for current iteration
        matching_rows = self._current_trajectory_data[self._current_trajectory_data['iteration'] == iteration_index]
        
        if matching_rows.empty:
            # Find closest iteration if exact match not found
            available_iterations = self._current_trajectory_data['iteration'].values
            import numpy as np
            closest_iteration_idx = np.argmin(np.abs(available_iterations - iteration_index))
            csv_start_idx = closest_iteration_idx
        else:
            csv_start_idx = matching_rows.index[0]
        
        # Calculate number of future states based on time horizon and sampling rate
        # This should match scenario.get_ego_future_trajectory() logic exactly
        dt_s = 0.05  # 20Hz = 0.05s intervals
        max_states_from_time = int(future_time_horizon / dt_s)
        
        # The number of future states is limited by both time horizon and num_poses
        # num_poses includes current state, so future states = num_poses - 1
        num_future_states = min(max_states_from_time, num_poses - 1)
        
        future_states = []
        base_timestamp = self._current_trajectory_data.iloc[csv_start_idx]['timestamp_us']
        
        for i in range(1, num_future_states + 1):  # Start from 1 (skip current state)
            future_idx = csv_start_idx + i
            
            if future_idx < len(self._current_trajectory_data):
                # Use actual CSV data
                row = self._current_trajectory_data.iloc[future_idx]
                
                # Create EgoState from CSV data with proper timestamp
                future_timestamp_us = base_timestamp + int(i * dt_s * 1e6)
                ego_state = self._create_ego_state_from_csv_row(row, future_timestamp_us)
                future_states.append(ego_state)
            else:
                # Extend with last available state if CSV data is insufficient
                if future_states:
                    last_state = future_states[-1]
                    extended_timestamp_us = base_timestamp + int(i * dt_s * 1e6)
                    extended_state = EgoState(
                        car_footprint=last_state.car_footprint,
                        dynamic_car_state=last_state.dynamic_car_state,
                        tire_steering_angle=last_state.tire_steering_angle,
                        is_in_auto_mode=last_state.is_in_auto_mode,
                        time_point=TimePoint(extended_timestamp_us)
                    )
                    future_states.append(extended_state)
                else:
                    # No future states available - this should raise AssertionError like scenario does
                    raise AssertionError("Insufficient CSV data for future trajectory")
        
        return future_states
    
    def _create_ego_state_from_csv_row(self, row: pd.Series, timestamp_us: int) -> EgoState:
        """
        Create EgoState from CSV row data.
        
        :param row: CSV row with trajectory data
        :param timestamp_us: Timestamp in microseconds
        :return: EgoState object
        """
        # Extract pose and velocity data
        x = float(row[f"{self.trajectory_column_prefix}x"])
        y = float(row[f"{self.trajectory_column_prefix}y"])
        heading = float(row[f"{self.trajectory_column_prefix}heading"])
        vx = float(row[f"{self.trajectory_column_prefix}velocity_x"])
        vy = float(row[f"{self.trajectory_column_prefix}velocity_y"])
        
        # Create state representation
        pose = StateSE2(x, y, heading)
        velocity = StateVector2D(vx, vy)
        time_point = TimePoint(timestamp_us)
        
        # Create dynamic car state
        vehicle_params = get_pacifica_parameters()
        rear_axle_to_center_dist = vehicle_params.rear_axle_to_center
        acceleration = StateVector2D(0.0, 0.0)  # Default acceleration
        
        dynamic_state = DynamicCarState.build_from_rear_axle(
            rear_axle_to_center_dist=rear_axle_to_center_dist,
            rear_axle_velocity_2d=velocity,
            rear_axle_acceleration_2d=acceleration,
            angular_velocity=0.0,  # Default
            angular_acceleration=0.0,  # Default
            tire_steering_rate=0.0  # Default
        )
        
        # Create car footprint
        car_footprint = CarFootprint.build_from_rear_axle(pose, vehicle_params)
        
        # Create and return EgoState
        return EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=0.0,
            is_in_auto_mode=True,
            time_point=time_point
        )

    def _csv_to_ego_states(self, csv_data: pd.DataFrame, base_time: TimePoint) -> List[EgoState]:
        """
        Convert CSV data to list of EgoState objects.
        
        :param csv_data: DataFrame with trajectory data
        :param base_time: Base time for trajectory
        :return: List of EgoState objects
        """
        ego_states = []
        
        for _, (_, row) in enumerate(csv_data.iterrows()):
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
            
            # Create timestamp aligned with nuPlan scenario
            iteration = int(row['iteration'])
            if self._scenario_start_time is not None:
                # Use scenario-aligned timestamp for proper open-loop evaluation
                timestamp_us = self._scenario_start_time + iteration * 250000
            else:
                # Fallback to base time if scenario alignment not available
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

    def _get_scenario_start_time(self) -> int:
        """
        Get the start time from the nuPlan scenario for timestamp alignment.
        
        :return: Start time in microseconds
        """
        if self._scenario is None:
            # Fallback to synthetic timestamp if no scenario available
            return 0
        
        try:
            # Get the first ego state from the scenario
            initial_ego_state = self._scenario.get_ego_state_at_iteration(0)
            start_time_us = initial_ego_state.time_point.time_us
            logger.info(f"Using scenario start time: {start_time_us} us")
            return start_time_us
        except Exception as e:
            logger.warning(f"Could not get scenario start time: {e}. Using synthetic timestamps.")
            return 0

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