"""
Mock SimulationHistory and related data structures for testing metrics.
"""

from dataclasses import dataclass
from typing import List, Optional
import numpy as np

from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.state_representation import StateSE2, Point2D, TimePoint, StateVector2D
from nuplan.common.actor_state.vehicle_parameters import VehicleParameters, get_pacifica_parameters
from nuplan.common.actor_state.dynamic_car_state import DynamicCarState
from nuplan.common.actor_state.car_footprint import CarFootprint
from nuplan.common.maps.abstract_map import AbstractMap
from nuplan.common.maps.maps_datatypes import TrafficLightStatusData
from nuplan.planning.simulation.observation.observation_type import Observation
from nuplan.planning.simulation.simulation_time_controller.simulation_iteration import SimulationIteration
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory
from nuplan.planning.simulation.history.simulation_history import SimulationHistory, SimulationHistorySample
from nuplan.planning.scenario_builder.test.mock_abstract_scenario import MockAbstractScenario


@dataclass(frozen=True)
class MockTrajectoryState:
    """Mock trajectory state for testing."""
    time_point: TimePoint
    pose: StateSE2


class MockTrajectory(AbstractTrajectory):
    """Mock trajectory implementation for testing."""
    
    def __init__(self, states: List[MockTrajectoryState]):
        self._states = states
        
    def get_sampled_trajectory(self) -> List[EgoState]:
        """Get sampled trajectory as ego states."""
        ego_states = []
        vehicle_params = get_pacifica_parameters()
        
        for state in self._states:
            # Create car footprint first to get rear_axle_to_center_dist
            car_footprint = CarFootprint.build_from_rear_axle(state.pose, vehicle_params)
            
            dynamic_state = DynamicCarState.build_from_rear_axle(
                rear_axle_to_center_dist=car_footprint.rear_axle_to_center_dist,
                rear_axle_velocity_2d=StateVector2D(5.0, 0.0),  # 5 m/s forward
                rear_axle_acceleration_2d=StateVector2D(0.0, 0.0),
                angular_velocity=0.0,
                angular_acceleration=0.0
            )
            
            ego_state = EgoState(
                car_footprint=car_footprint,
                dynamic_car_state=dynamic_state,
                tire_steering_angle=0.0,
                is_in_auto_mode=True,
                time_point=state.time_point
            )
            ego_states.append(ego_state)
            
        return ego_states
        
    def get_state_at_time(self, time_point: TimePoint) -> EgoState:
        """Get state at specific time point."""
        # Find closest state
        closest_state = min(self._states, key=lambda s: abs(s.time_point.time_us - time_point.time_us))
        
        vehicle_params = get_pacifica_parameters()
        car_footprint = CarFootprint.build_from_rear_axle(closest_state.pose, vehicle_params)
        
        dynamic_state = DynamicCarState.build_from_rear_axle(
            rear_axle_to_center_dist=car_footprint.rear_axle_to_center_dist,
            rear_axle_velocity_2d=StateVector2D(5.0, 0.0),
            rear_axle_acceleration_2d=StateVector2D(0.0, 0.0),
            angular_velocity=0.0,
            angular_acceleration=0.0
        )
        
        return EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=0.0,
            is_in_auto_mode=True,
            time_point=time_point
        )
    
    def get_state_at_times(self, time_points: List[TimePoint]) -> List[EgoState]:
        """Get states at multiple time points."""
        return [self.get_state_at_time(tp) for tp in time_points]
    
    @property
    def start_time(self) -> TimePoint:
        """Get start time."""
        return self._states[0].time_point
        
    @property
    def end_time(self) -> TimePoint:
        """Get end time."""
        return self._states[-1].time_point


class MockObservation(Observation):
    """Mock observation for testing."""
    pass




class MockScenario(MockAbstractScenario):
    """Mock scenario for testing planner-expert metrics."""
    
    def __init__(
        self,
        scenario_name: str = "test_scenario",
        scenario_type: str = "test_type", 
        log_name: str = "test_log",
        expert_ego_trajectory: Optional[List[EgoState]] = None,
        database_interval: float = 0.1
    ):
        # Initialize with default MockAbstractScenario parameters (minimal setup)
        super().__init__(
            number_of_detections=0,
            time_step=database_interval
        )
        self._scenario_name = scenario_name
        self._scenario_type = scenario_type
        self._log_name = log_name
        self._expert_ego_trajectory = expert_ego_trajectory or []
        self._database_interval = database_interval
        
    @property
    def scenario_name(self) -> str:
        return self._scenario_name
        
    @property
    def scenario_type(self) -> str:
        return self._scenario_type
        
    @property
    def log_name(self) -> str:
        return self._log_name
        
    @property
    def database_interval(self) -> float:
        return self._database_interval
    
    def get_expert_ego_trajectory(self) -> List[EgoState]:
        """Get expert trajectory."""
        return self._expert_ego_trajectory
        
    def get_ego_future_trajectory(self, iteration_index: int, time_horizon: float, num_samples: int) -> List[EgoState]:
        """Get future expert trajectory."""
        # Return subset of expert trajectory based on iteration
        start_idx = min(iteration_index + 1, len(self._expert_ego_trajectory))
        end_idx = min(start_idx + num_samples, len(self._expert_ego_trajectory))
        return self._expert_ego_trajectory[start_idx:end_idx]
    
    def get_mission_goal(self) -> StateSE2:
        """Get mission goal."""
        if self._expert_ego_trajectory:
            final_state = self._expert_ego_trajectory[-1]
            return final_state.rear_axle
        return StateSE2(100.0, 100.0, 0.0)  # Default goal


def create_mock_simulation_history(
    num_samples: int = 50,
    dt: float = 0.1,
    trajectory_noise: float = 0.1
) -> tuple[SimulationHistory, MockScenario]:
    """
    Create a mock SimulationHistory with planner and expert trajectories for testing.
    
    Args:
        num_samples: Number of simulation samples
        dt: Time step between samples
        trajectory_noise: Noise level for planner trajectory vs expert
        
    Returns:
        Tuple of (SimulationHistory, MockScenario)
    """
    # Create expert trajectory (straight line with slight curve)
    expert_states = []
    vehicle_params = get_pacifica_parameters()
    
    for i in range(num_samples):
        time_us = int(i * dt * 1e6)  # Convert to microseconds
        x = i * 5.0  # 5 meters per step
        y = 2.0 * np.sin(i * 0.1)  # Slight curve
        heading = 0.1 * np.cos(i * 0.1)  # Varying heading
        
        pose = StateSE2(x, y, heading)
        car_footprint = CarFootprint.build_from_rear_axle(pose, vehicle_params)
        
        dynamic_state = DynamicCarState.build_from_rear_axle(
            rear_axle_to_center_dist=car_footprint.rear_axle_to_center_dist,
            rear_axle_velocity_2d=StateVector2D(5.0, 0.0),
            rear_axle_acceleration_2d=StateVector2D(0.0, 0.0),
            angular_velocity=0.0,
            angular_acceleration=0.0
        )
        
        ego_state = EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=0.0,
            is_in_auto_mode=True,
            time_point=TimePoint(time_us)
        )
        expert_states.append(ego_state)
    
    # Create scenario with expert trajectory
    scenario = MockScenario(
        expert_ego_trajectory=expert_states,
        database_interval=dt
    )
    
    # Create simulation history with planner trajectories that deviate from expert
    history = SimulationHistory(scenario.map_api, scenario.get_mission_goal())
    
    for i in range(num_samples):
        # Current ego state (same as expert for simplicity)
        ego_state = expert_states[i]
        
        # Create planner trajectory for future horizon (with noise)
        future_states = []
        horizon_steps = min(50, num_samples - i - 1)  # 5 second horizon at 10Hz
        
        for j in range(1, horizon_steps + 1):
            future_idx = i + j
            if future_idx < len(expert_states):
                expert_pose = expert_states[future_idx].rear_axle
                
                # Add noise to planner trajectory
                noise_x = np.random.normal(0, trajectory_noise)
                noise_y = np.random.normal(0, trajectory_noise)
                noise_heading = np.random.normal(0, trajectory_noise * 0.1)
                
                planner_pose = StateSE2(
                    expert_pose.x + noise_x,
                    expert_pose.y + noise_y,
                    expert_pose.heading + noise_heading
                )
                
                future_states.append(MockTrajectoryState(
                    time_point=expert_states[future_idx].time_point,
                    pose=planner_pose
                ))
        
        # Create trajectory
        if future_states:
            trajectory = MockTrajectory(future_states)
        else:
            # Fallback for last samples
            trajectory = MockTrajectory([MockTrajectoryState(
                time_point=TimePoint(int((i + 1) * dt * 1e6)),
                pose=ego_state.rear_axle
            )])
        
        # Create simulation sample
        iteration = SimulationIteration(TimePoint(ego_state.time_point.time_us), i)
        observation = MockObservation()
        traffic_lights: List[TrafficLightStatusData] = []
        
        sample = SimulationHistorySample(
            iteration=iteration,
            ego_state=ego_state,
            trajectory=trajectory,
            observation=observation,
            traffic_light_status=traffic_lights
        )
        
        history.add_sample(sample)
    
    return history, scenario