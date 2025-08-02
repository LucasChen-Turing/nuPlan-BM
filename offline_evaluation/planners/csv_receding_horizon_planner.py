#!/usr/bin/env python3
"""
CSV Receding Horizon Planner

A planner that implements proper receding horizon behavior for Challenge 1 metrics.
At each timestep, it outputs future trajectory predictions that get evaluated against expert trajectories.
"""

import itertools
import json
import logging
import pandas as pd
import numpy as np
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


class CSVRecedingHorizonPlanner(AbstractPlanner):
    """
    A planner that implements receding horizon behavior for Challenge 1 metrics.
    
    At each timestep (1Hz), this planner outputs future trajectory predictions
    extending 6+ seconds ahead. These predictions are compared against the actual
    expert trajectory by the Challenge 1 "within bound" metrics (ADE, FDE, Miss Rate, AHE, FHE).
    """

    # Inherited property, see superclass.
    requires_scenario: bool = True

    def __init__(
        self, 
        csv_file_path: str,
        scenario=None,
        num_poses: int = 120,  # 6 seconds * 20Hz = 120 poses minimum for Challenge 1
        future_time_horizon: float = 8.0,  # Must be >= 6.0 for Challenge 1
        scenario_id_column: str = "scenario_id",
        trajectory_column_prefix: str = "ego_",
        planning_frequency_hz: float = 1.0  # Challenge 1 requires 1Hz planning frequency
    ):
        """
        Constructor for CSVRecedingHorizonPlanner.
        
        :param csv_file_path: Path to CSV file containing trajectory data
        :param num_poses: Number of poses in receding horizon (must be >= 120 for 6s at 20Hz)
        :param future_time_horizon: Time horizon for trajectory (must be >= 6.0s for Challenge 1)
        :param scenario_id_column: Column name for scenario identification
        :param trajectory_column_prefix: Prefix for trajectory columns
        :param planning_frequency_hz: Planning frequency (Challenge 1 requires 1Hz)
        """
        super().__init__()
        
        # Validate Challenge 1 requirements
        if future_time_horizon < 6.0:
            raise ValueError(f"future_time_horizon must be >= 6.0s for Challenge 1, got {future_time_horizon}")
        
        if num_poses < 120:
            logger.warning(f"num_poses {num_poses} may be insufficient for Challenge 1 (need >=120 for 6s at 20Hz)")
        
        # Store scenario
        self._scenario = scenario
        self._scenario_start_time: Optional[int] = None
        
        self.csv_file_path = Path(csv_file_path)
        self._num_poses = num_poses
        self._future_time_horizon = future_time_horizon
        self.scenario_id_column = scenario_id_column
        self.trajectory_column_prefix = trajectory_column_prefix
        self._planning_frequency_hz = planning_frequency_hz
        
        # Calculate planning interval for 1Hz frequency
        self._planning_interval_s = 1.0 / planning_frequency_hz
        self._data_frequency_hz = 20.0  # Assume CSV data is at 20Hz
        self._data_interval_s = 1.0 / self._data_frequency_hz
        self._planning_step_size = int(self._planning_interval_s / self._data_interval_s)  # Steps between planning cycles
        
        # Vehicle parameters
        self.vehicle_parameters = get_pacifica_parameters()
        
        # Load CSV data
        self._load_trajectory_data()
        
        # Current state
        self._current_scenario_id: Optional[str] = None
        self._current_trajectory_data: Optional[pd.DataFrame] = None
        self._trajectory: Optional[AbstractTrajectory] = None
        
        logger.info(f"Initialized receding horizon planner: {future_time_horizon}s horizon, {num_poses} poses, {planning_frequency_hz}Hz frequency")

    def _load_trajectory_data(self) -> None:
        """Load trajectory data from CSV file."""
        if not self.csv_file_path.exists():
            raise FileNotFoundError(f"CSV trajectory file not found: {self.csv_file_path}")
        
        try:
            self.trajectory_df = pd.read_csv(self.csv_file_path)
            logger.info(f"Loaded trajectory data: {len(self.trajectory_df)} rows")
            
            # Validate required columns for enhanced CSV format
            required_columns = [
                # Current state columns
                f"{self.trajectory_column_prefix}x",
                f"{self.trajectory_column_prefix}y", 
                f"{self.trajectory_column_prefix}heading",
                f"{self.trajectory_column_prefix}velocity_x",
                f"{self.trajectory_column_prefix}velocity_y",
                "timestamp_us",
                "planning_iteration",
                self.scenario_id_column,
                # Horizon data columns (JSON arrays)
                "horizon_x",
                "horizon_y",
                "horizon_heading",
                "horizon_velocity_x",
                "horizon_velocity_y",
                "horizon_seconds",
                "horizon_length"
            ]
            
            missing_columns = [col for col in required_columns if col not in self.trajectory_df.columns]
            if missing_columns:
                raise ValueError(f"Missing required columns in CSV: {missing_columns}")
            
            # Group trajectories by scenario ID
            self.scenario_trajectories = {}
            for scenario_id, group in self.trajectory_df.groupby(self.scenario_id_column):
                # Sort by planning_iteration to ensure correct order
                sorted_group = group.sort_values('planning_iteration').reset_index(drop=True)
                self.scenario_trajectories[scenario_id] = sorted_group
                
            logger.info(f"Loaded {len(self.scenario_trajectories)} unique scenarios")
            
        except Exception as e:
            raise RuntimeError(f"Failed to load CSV trajectory data: {e}")

    def set_current_scenario(self, scenario_id: str) -> None:
        """Set the current scenario to extract trajectory from."""
        if scenario_id not in self.scenario_trajectories:
            available_scenarios = list(self.scenario_trajectories.keys())
            raise ValueError(f"Scenario '{scenario_id}' not found. Available: {available_scenarios}")
        
        self._current_scenario_id = scenario_id
        self._current_trajectory_data = self.scenario_trajectories[scenario_id]
        logger.info(f"Set current scenario: {scenario_id} ({len(self._current_trajectory_data)} points)")
        
        # Get scenario start time for timestamp alignment
        self._scenario_start_time = self._get_scenario_start_time()

    def _get_scenario_start_time(self) -> Optional[int]:
        """Get the start time of the current scenario from nuPlan."""
        if self._scenario is not None:
            try:
                initial_iteration = self._scenario.get_number_of_iterations() - len(self._current_trajectory_data)
                if initial_iteration < 0:
                    initial_iteration = 0
                initial_time_point = self._scenario.start_time
                return initial_time_point.time_us + initial_iteration * 50000  # 20Hz = 50ms intervals
            except Exception as e:
                logger.warning(f"Could not get scenario start time: {e}")
        return None

    def initialize(self, initialization: PlannerInitialization) -> None:
        """Inherited, see superclass."""
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
        """
        Inherited, see superclass.
        
        Implements receding horizon planning for Challenge 1 metrics:
        - At each timestep, output future trajectory prediction extending 6+ seconds
        - These predictions get compared against actual expert trajectory by metrics
        """
        current_iteration = current_input.iteration.index
        current_state = self._scenario.get_ego_state_at_iteration(current_iteration)
        
        try:
            # Get receding horizon trajectory prediction
            future_states = self._get_receding_horizon_trajectory(current_iteration)
            
            # Create trajectory including current state + future predictions (Challenge 1 requirement)
            # Must include current state + 6+ future states for proper horizon evaluation
            all_states = [current_state] + future_states
            self._trajectory = InterpolatedTrajectory(all_states)
            
            logger.debug(f"Generated receding horizon trajectory at iteration {current_iteration}: "
                        f"{len(all_states)} total states (1 current + {len(future_states)} future), {self._future_time_horizon}s horizon")
                        
        except Exception as e:
            logger.warning(f"Cannot retrieve receding horizon trajectory at iteration {current_iteration}: {e}")
            if self._trajectory is None:
                raise RuntimeError("Receding horizon trajectory cannot be retrieved!")

        return self._trajectory
    
    def _get_receding_horizon_trajectory(self, current_iteration: int) -> List[EgoState]:
        """
        Get receding horizon trajectory prediction from enhanced CSV data.
        
        This implements the core Challenge 1 behavior:
        - At current timestep, extract the pre-stored 6+ second horizon prediction
        - This prediction gets compared against actual expert trajectory by metrics
        
        :param current_iteration: Current iteration index in simulation
        :return: List of predicted future ego states
        """
        if self._current_trajectory_data is None:
            raise RuntimeError("No trajectory data loaded. Call set_current_scenario() first.")
        
        # Map simulation iteration to planning iteration (1Hz intervals)
        planning_iteration = self._map_simulation_to_planning_iteration(current_iteration)
        
        # Find the CSV row for this planning iteration
        matching_rows = self._current_trajectory_data[self._current_trajectory_data['planning_iteration'] == planning_iteration]
        
        if matching_rows.empty:
            # Find closest planning iteration if exact match not found
            available_iterations = self._current_trajectory_data['planning_iteration'].values
            if len(available_iterations) == 0:
                raise RuntimeError("No planning iterations available in CSV data")
            
            closest_idx = np.argmin(np.abs(available_iterations - planning_iteration))
            planning_row = self._current_trajectory_data.iloc[closest_idx]
            actual_planning_iteration = available_iterations[closest_idx]
            logger.debug(f"No exact match for planning iteration {planning_iteration}, using closest: {actual_planning_iteration}")
        else:
            planning_row = matching_rows.iloc[0]
            actual_planning_iteration = planning_iteration
        
        # Extract horizon data from JSON arrays
        try:
            horizon_x = json.loads(planning_row['horizon_x'])
            horizon_y = json.loads(planning_row['horizon_y'])
            horizon_heading = json.loads(planning_row['horizon_heading'])
            horizon_vx = json.loads(planning_row['horizon_velocity_x'])
            horizon_vy = json.loads(planning_row['horizon_velocity_y'])
            horizon_ax = json.loads(planning_row['horizon_acceleration_x'])
            horizon_ay = json.loads(planning_row['horizon_acceleration_y'])
            
            horizon_length = len(horizon_x)
            horizon_seconds = float(planning_row['horizon_seconds'])
            
            logger.debug(f"Extracted horizon for planning iteration {actual_planning_iteration}: "
                        f"{horizon_length} points, {horizon_seconds}s duration")
            
        except (KeyError, json.JSONDecodeError) as e:
            raise RuntimeError(f"Failed to extract horizon data from CSV row: {e}")
        
        # Validate horizon data
        if not all(len(arr) == horizon_length for arr in [horizon_y, horizon_heading, horizon_vx, horizon_vy, horizon_ax, horizon_ay]):
            raise RuntimeError("Inconsistent horizon array lengths in CSV data")
        
        if horizon_seconds < 6.0:
            logger.warning(f"Horizon duration {horizon_seconds}s is less than 6s required for Challenge 1")
        
        # Convert horizon data to EgoState objects
        future_states = []
        base_timestamp = int(planning_row['timestamp_us'])
        dt_s = self._data_interval_s  # 0.05s for 20Hz
        
        # For Challenge 1, we need exactly 6 future states minimum (plus current = 7 total)
        # Ensure we provide at least 6 future prediction points for proper horizon evaluation
        min_challenge1_future_states = int(6.0 / dt_s)  # 6 seconds at 20Hz = 120 states
        
        # Limit horizon to requested time and poses, but ensure minimum for Challenge 1
        max_horizon_states = min(
            horizon_length,
            int(self._future_time_horizon / dt_s),
            self._num_poses  # Don't subtract 1 - we want full 120 future states for Challenge 1
        )
        
        # Ensure we have at least the minimum required for Challenge 1
        if max_horizon_states < min_challenge1_future_states:
            logger.warning(f"Horizon states {max_horizon_states} < Challenge 1 minimum {min_challenge1_future_states}")
            max_horizon_states = min(horizon_length, min_challenge1_future_states)
        
        for i in range(max_horizon_states):
            # Create timestamp for this horizon point
            horizon_timestamp_us = base_timestamp + int((i + 1) * dt_s * 1e6)
            
            # Create EgoState from horizon data
            ego_state = self._create_ego_state_from_horizon_data(
                x=horizon_x[i],
                y=horizon_y[i],
                heading=horizon_heading[i],
                vx=horizon_vx[i],
                vy=horizon_vy[i],
                ax=horizon_ax[i],
                ay=horizon_ay[i],
                timestamp_us=horizon_timestamp_us
            )
            future_states.append(ego_state)
        
        logger.debug(f"Generated {len(future_states)} horizon states from planning iteration {actual_planning_iteration}")
        
        return future_states
    
    def _map_simulation_to_planning_iteration(self, simulation_iteration: int) -> int:
        """
        Map simulation iteration to planning iteration for enhanced CSV format.
        
        The enhanced CSV has planning iterations at 1Hz intervals (every 20 simulation steps).
        """
        # Convert simulation iteration (20Hz) to planning iteration (1Hz)
        planning_iteration = simulation_iteration // self._planning_step_size
        return planning_iteration
    
    def _create_ego_state_from_horizon_data(self, x: float, y: float, heading: float, 
                                          vx: float, vy: float, ax: float, ay: float,
                                          timestamp_us: int) -> EgoState:
        """Create EgoState from horizon prediction data."""
        # Create state representations
        pose = StateSE2(x, y, heading)
        velocity = StateVector2D(vx, vy)
        acceleration = StateVector2D(ax, ay)
        time_point = TimePoint(timestamp_us)
        
        # Create dynamic car state
        rear_axle_to_center_dist = self.vehicle_parameters.rear_axle_to_center
        
        dynamic_state = DynamicCarState.build_from_rear_axle(
            rear_axle_to_center_dist=rear_axle_to_center_dist,
            rear_axle_velocity_2d=velocity,
            rear_axle_acceleration_2d=acceleration,
            angular_velocity=0.0,  # Default - could be computed from heading changes
            angular_acceleration=0.0,  # Default
            tire_steering_rate=0.0  # Default
        )
        
        # Create car footprint
        car_footprint = CarFootprint.build_from_rear_axle(pose, self.vehicle_parameters)
        
        return EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=0.0,  # Default
            is_in_auto_mode=True,
            time_point=time_point
        )
    
    def _create_ego_state_from_csv_row(self, row: pd.Series, timestamp_us: int) -> EgoState:
        """Create EgoState from CSV row data."""
        # Extract pose and velocity data
        x = float(row[f"{self.trajectory_column_prefix}x"])
        y = float(row[f"{self.trajectory_column_prefix}y"])
        heading = float(row[f"{self.trajectory_column_prefix}heading"])
        vx = float(row[f"{self.trajectory_column_prefix}velocity_x"])
        vy = float(row[f"{self.trajectory_column_prefix}velocity_y"])
        
        # Extract optional acceleration data
        ax = float(row.get(f"{self.trajectory_column_prefix}acceleration_x", 0.0))
        ay = float(row.get(f"{self.trajectory_column_prefix}acceleration_y", 0.0))
        
        # Create state representations
        pose = StateSE2(x, y, heading)
        velocity = StateVector2D(vx, vy)
        acceleration = StateVector2D(ax, ay)
        time_point = TimePoint(timestamp_us)
        
        # Create dynamic car state
        rear_axle_to_center_dist = self.vehicle_parameters.rear_axle_to_center
        
        dynamic_state = DynamicCarState.build_from_rear_axle(
            rear_axle_to_center_dist=rear_axle_to_center_dist,
            rear_axle_velocity_2d=velocity,
            rear_axle_acceleration_2d=acceleration,
            angular_velocity=0.0,  # Default - could be extracted from CSV if available
            angular_acceleration=0.0,  # Default
            tire_steering_rate=0.0  # Default
        )
        
        # Create car footprint
        car_footprint = CarFootprint.build_from_rear_axle(pose, self.vehicle_parameters)
        
        return EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=float(row.get("tire_steering_angle", 0.0)),
            is_in_auto_mode=True,
            time_point=time_point
        )