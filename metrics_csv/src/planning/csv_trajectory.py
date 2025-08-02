#!/usr/bin/env python3
"""
CSV Trajectory Implementation for nuPlan Integration

This module provides trajectory classes that convert CSV planner data
into nuPlan-compatible trajectory objects for metrics evaluation.

Author: Lucas, C., Claude Agent
Date: August 2025
"""

import numpy as np
import pandas as pd
from typing import List, Dict, Any, Optional, Union
from dataclasses import dataclass

from nuplan.common.actor_state.state_representation import TimePoint
from nuplan.common.actor_state.ego_state import EgoState
from nuplan.common.actor_state.car_footprint import CarFootprint
from nuplan.common.actor_state.dynamic_car_state import DynamicCarState
from nuplan.common.actor_state.state_representation import StateSE2
from nuplan.common.actor_state.vehicle_parameters import get_pacifica_parameters
from nuplan.planning.simulation.trajectory.abstract_trajectory import AbstractTrajectory

from .csv_data_analyzer import CSVColumnMapping


@dataclass
class CSVTrajectoryPoint:
    """Single trajectory point from CSV data."""
    
    timestamp: float
    x: float
    y: float
    heading: float
    velocity_x: Optional[float] = None
    velocity_y: Optional[float] = None
    acceleration_x: Optional[float] = None
    acceleration_y: Optional[float] = None
    steering_angle: Optional[float] = None


class CSVTrajectory(AbstractTrajectory):
    """
    Trajectory implementation using CSV planner data.
    
    This class converts CSV trajectory/horizon data into nuPlan-compatible
    trajectory objects for metrics evaluation.
    """
    
    def __init__(
        self,
        trajectory_points: List[CSVTrajectoryPoint],
        start_time_offset: float = 0.0
    ):
        """
        Initialize trajectory from CSV data points.
        
        Args:
            trajectory_points: List of trajectory points from CSV
            start_time_offset: Time offset to add to all timestamps
        """
        if not trajectory_points:
            raise ValueError("Trajectory must contain at least one point")
        
        self._trajectory_points = sorted(trajectory_points, key=lambda p: p.timestamp)
        self._start_time_offset = start_time_offset
        self._vehicle_params = get_pacifica_parameters()
        
        # Convert to EgoState objects
        self._ego_states = self._convert_to_ego_states()
    
    def _convert_to_ego_states(self) -> List[EgoState]:
        """Convert CSV trajectory points to EgoState objects."""
        ego_states = []
        
        for point in self._trajectory_points:
            # Create time point
            time_point = TimePoint(int((point.timestamp + self._start_time_offset) * 1e6))
            
            # Create pose (rear axle position)
            rear_axle_pose = StateSE2(x=point.x, y=point.y, heading=point.heading)
            
            # Create dynamic state
            velocity_x = point.velocity_x if point.velocity_x is not None else 0.0
            velocity_y = point.velocity_y if point.velocity_y is not None else 0.0
            acceleration_x = point.acceleration_x if point.acceleration_x is not None else 0.0
            acceleration_y = point.acceleration_y if point.acceleration_y is not None else 0.0
            
            dynamic_state = DynamicCarState(
                rear_axle_to_center_dist=self._vehicle_params.rear_axle_to_center,
                rear_axle_velocity_2d=StateSE2(x=velocity_x, y=velocity_y, heading=0.0),
                rear_axle_acceleration_2d=StateSE2(x=acceleration_x, y=acceleration_y, heading=0.0),
                angular_velocity=0.0,  # Could be computed from heading changes
                angular_acceleration=0.0
            )
            
            # Create car footprint
            car_footprint = CarFootprint.build_from_rear_axle(
                rear_axle_pose=rear_axle_pose,
                vehicle_parameters=self._vehicle_params
            )
            
            # Create ego state
            ego_state = EgoState(
                car_footprint=car_footprint,
                dynamic_car_state=dynamic_state,
                tire_steering_angle=point.steering_angle if point.steering_angle is not None else 0.0,
                is_in_auto_mode=True,
                time_point=time_point
            )
            
            ego_states.append(ego_state)
        
        return ego_states
    
    @property
    def start_time(self) -> TimePoint:
        """Get trajectory start time."""
        return self._ego_states[0].time_point
    
    @property
    def end_time(self) -> TimePoint:
        """Get trajectory end time."""
        return self._ego_states[-1].time_point
    
    def get_state_at_time(self, time_point: TimePoint) -> EgoState:
        """
        Get ego state at specified time point using interpolation.
        
        Args:
            time_point: Time for which to query state
            
        Returns:
            EgoState at the specified time
        """
        # Make range check more tolerant and add extrapolation
        tolerance_us = 50000  # 50ms tolerance
        
        if len(self._ego_states) == 0:
            raise AssertionError("Trajectory has no states")
        
        start_time = self._ego_states[0].time_point.time_us
        end_time = self._ego_states[-1].time_point.time_us
        
        # Debug output for problematic queries
        if time_point.time_us < start_time or time_point.time_us > end_time:
            print(f"   🔍 Query time: {time_point.time_us}, Range: {start_time} to {end_time}")
        
        # Allow extrapolation beyond trajectory bounds
        if time_point.time_us < (start_time - tolerance_us):
            print(f"   ⚠️  Extrapolating before start: returning first state")
            return self._ego_states[0]
        elif time_point.time_us > (end_time + tolerance_us):
            print(f"   ⚠️  Extrapolating beyond end: returning last state") 
            return self._ego_states[-1]
        
        # Find surrounding states for interpolation
        for i, state in enumerate(self._ego_states):
            if state.time_point >= time_point:
                if i == 0 or state.time_point == time_point:
                    return state
                
                # Interpolate between states
                prev_state = self._ego_states[i - 1]
                return self._interpolate_states(prev_state, state, time_point)
        
        # Should not reach here due to range check, but return last state as fallback
        return self._ego_states[-1]
    
    def get_state_at_times(self, time_points: List[TimePoint]) -> List[EgoState]:
        """Get ego states at specified time points."""
        return [self.get_state_at_time(tp) for tp in time_points]
    
    def get_sampled_trajectory(self) -> List[EgoState]:
        """Get the sampled states along the trajectory."""
        return self._ego_states.copy()
    
    def _interpolate_states(self, state1: EgoState, state2: EgoState, target_time: TimePoint) -> EgoState:
        """
        Interpolate between two ego states.
        
        Args:
            state1: Earlier state
            state2: Later state
            target_time: Target time for interpolation
            
        Returns:
            Interpolated EgoState
        """
        # Calculate interpolation factor
        dt_total = state2.time_point.time_s - state1.time_point.time_s
        dt_target = target_time.time_s - state1.time_point.time_s
        alpha = dt_target / dt_total if dt_total > 0 else 0.0
        
        # Interpolate position
        pose1 = state1.rear_axle
        pose2 = state2.rear_axle
        
        interp_x = pose1.x + alpha * (pose2.x - pose1.x)
        interp_y = pose1.y + alpha * (pose2.y - pose1.y)
        
        # Interpolate heading (handle angle wraparound)
        heading_diff = pose2.heading - pose1.heading
        if heading_diff > np.pi:
            heading_diff -= 2 * np.pi
        elif heading_diff < -np.pi:
            heading_diff += 2 * np.pi
        interp_heading = pose1.heading + alpha * heading_diff
        
        # Create interpolated pose
        interp_pose = StateSE2(x=interp_x, y=interp_y, heading=interp_heading)
        
        # Interpolate velocities
        vel1 = state1.dynamic_car_state.rear_axle_velocity_2d
        vel2 = state2.dynamic_car_state.rear_axle_velocity_2d
        interp_vel = StateSE2(
            x=vel1.x + alpha * (vel2.x - vel1.x),
            y=vel1.y + alpha * (vel2.y - vel1.y),
            heading=0.0
        )
        
        # Interpolate accelerations
        acc1 = state1.dynamic_car_state.rear_axle_acceleration_2d
        acc2 = state2.dynamic_car_state.rear_axle_acceleration_2d
        interp_acc = StateSE2(
            x=acc1.x + alpha * (acc2.x - acc1.x),
            y=acc1.y + alpha * (acc2.y - acc1.y),
            heading=0.0
        )
        
        # Create interpolated dynamic state
        interp_dynamic_state = DynamicCarState(
            rear_axle_to_center_dist=self._vehicle_params.rear_axle_to_center,
            rear_axle_velocity_2d=interp_vel,
            rear_axle_acceleration_2d=interp_acc,
            angular_velocity=0.0,
            angular_acceleration=0.0
        )
        
        # Create interpolated car footprint
        interp_footprint = CarFootprint.build_from_rear_axle(
            rear_axle_pose=interp_pose,
            vehicle_parameters=self._vehicle_params
        )
        
        # Interpolate steering angle
        interp_steering = state1.tire_steering_angle + alpha * (state2.tire_steering_angle - state1.tire_steering_angle)
        
        # Create interpolated ego state
        return EgoState(
            car_footprint=interp_footprint,
            dynamic_car_state=interp_dynamic_state,
            tire_steering_angle=interp_steering,
            is_in_auto_mode=True,
            time_point=target_time
        )


class CSVEgoStateConverter:
    """
    Converter class for transforming CSV data rows into EgoState objects.
    """
    
    def __init__(self, column_mapping: CSVColumnMapping):
        """
        Initialize converter with column mapping.
        
        Args:
            column_mapping: Mapping of CSV columns to data fields
        """
        self.mapping = column_mapping
        self._vehicle_params = get_pacifica_parameters()
    
    def convert_row_to_ego_state(
        self, 
        csv_row: Union[Dict[str, Any], pd.Series],
        base_timestamp: Optional[float] = None
    ) -> EgoState:
        """
        Convert a single CSV row to EgoState.
        
        Args:
            csv_row: CSV row data (dict or pandas Series)
            base_timestamp: Base timestamp if not in CSV
            
        Returns:
            EgoState object
        """
        # Extract data from row
        if isinstance(csv_row, pd.Series):
            row_data = csv_row.to_dict()
        else:
            row_data = csv_row
        
        # Extract timestamp
        if self.mapping.timestamp_col and self.mapping.timestamp_col in row_data:
            timestamp = float(row_data[self.mapping.timestamp_col])
        elif base_timestamp is not None:
            timestamp = base_timestamp
        else:
            timestamp = 0.0
        
        # Extract position
        x = float(row_data.get(self.mapping.x_col, 0.0))
        y = float(row_data.get(self.mapping.y_col, 0.0))
        heading = float(row_data.get(self.mapping.heading_col, 0.0))
        
        # Extract velocities
        velocity_x = float(row_data.get(self.mapping.velocity_x_col, 0.0)) if self.mapping.velocity_x_col else 0.0
        velocity_y = float(row_data.get(self.mapping.velocity_y_col, 0.0)) if self.mapping.velocity_y_col else 0.0
        
        # Extract accelerations
        acc_x = float(row_data.get(self.mapping.acceleration_x_col, 0.0)) if self.mapping.acceleration_x_col else 0.0
        acc_y = float(row_data.get(self.mapping.acceleration_y_col, 0.0)) if self.mapping.acceleration_y_col else 0.0
        
        # Extract steering
        steering = float(row_data.get(self.mapping.steering_angle_col, 0.0)) if self.mapping.steering_angle_col else 0.0
        
        # Create trajectory point
        trajectory_point = CSVTrajectoryPoint(
            timestamp=timestamp,
            x=x,
            y=y,
            heading=heading,
            velocity_x=velocity_x,
            velocity_y=velocity_y,
            acceleration_x=acc_x,
            acceleration_y=acc_y,
            steering_angle=steering
        )
        
        # Convert to ego state
        time_point = TimePoint(int(timestamp * 1e6))
        rear_axle_pose = StateSE2(x=x, y=y, heading=heading)
        
        dynamic_state = DynamicCarState(
            rear_axle_to_center_dist=self._vehicle_params.rear_axle_to_center,
            rear_axle_velocity_2d=StateSE2(x=velocity_x, y=velocity_y, heading=0.0),
            rear_axle_acceleration_2d=StateSE2(x=acc_x, y=acc_y, heading=0.0),
            angular_velocity=0.0,
            angular_acceleration=0.0
        )
        
        car_footprint = CarFootprint.build_from_rear_axle(
            rear_axle_pose=rear_axle_pose,
            vehicle_parameters=self._vehicle_params
        )
        
        return EgoState(
            car_footprint=car_footprint,
            dynamic_car_state=dynamic_state,
            tire_steering_angle=steering,
            is_in_auto_mode=True,
            time_point=time_point
        )
    
    def convert_horizon_to_trajectory(
        self,
        csv_row: Union[Dict[str, Any], pd.Series],
        base_timestamp: float,
        dt: float = 0.1
    ) -> CSVTrajectory:
        """
        Convert horizon data from CSV row to trajectory.
        
        Args:
            csv_row: CSV row containing horizon data
            base_timestamp: Starting timestamp for horizon
            dt: Time step between horizon points
            
        Returns:
            CSVTrajectory object
        """
        if not self.mapping.horizon_prefix:
            raise ValueError("No horizon prefix configured in mapping")
        
        # Extract data from row
        if isinstance(csv_row, pd.Series):
            row_data = csv_row.to_dict()
        else:
            row_data = csv_row
        
        # Find horizon columns
        horizon_cols = [col for col in row_data.keys() if col.startswith(self.mapping.horizon_prefix)]
        
        if not horizon_cols:
            raise ValueError(f"No horizon columns found with prefix '{self.mapping.horizon_prefix}'")
        
        # Group columns by coordinate (x, y, heading, etc.)
        coord_groups = {}
        for col in horizon_cols:
            # Extract coordinate type and index
            # Expected format: prefix_coord_index (e.g., "horizon_x_0", "horizon_y_1")
            parts = col.split('_')
            if len(parts) >= 3:
                coord_type = parts[-2]
                index = int(parts[-1])
                
                if coord_type not in coord_groups:
                    coord_groups[coord_type] = {}
                coord_groups[coord_type][index] = row_data[col]
        
        # Create trajectory points
        trajectory_points = []
        max_index = max(max(indices.keys()) for indices in coord_groups.values()) if coord_groups else 0
        
        for i in range(max_index + 1):
            timestamp = base_timestamp + i * dt
            
            # Extract coordinates for this index
            x = coord_groups.get('x', {}).get(i, 0.0)
            y = coord_groups.get('y', {}).get(i, 0.0)
            heading = coord_groups.get('heading', {}).get(i, 0.0)
            vel_x = coord_groups.get('vx', {}).get(i, None)
            vel_y = coord_groups.get('vy', {}).get(i, None)
            acc_x = coord_groups.get('ax', {}).get(i, None)
            acc_y = coord_groups.get('ay', {}).get(i, None)
            steering = coord_groups.get('steering', {}).get(i, None)
            
            point = CSVTrajectoryPoint(
                timestamp=timestamp,
                x=float(x),
                y=float(y),
                heading=float(heading),
                velocity_x=float(vel_x) if vel_x is not None else None,
                velocity_y=float(vel_y) if vel_y is not None else None,
                acceleration_x=float(acc_x) if acc_x is not None else None,
                acceleration_y=float(acc_y) if acc_y is not None else None,
                steering_angle=float(steering) if steering is not None else None
            )
            
            trajectory_points.append(point)
        
        return CSVTrajectory(trajectory_points)


def main():
    """Main function for testing trajectory conversion."""
    print("CSV Trajectory Converter - Test Mode")
    print("This module provides CSV to nuPlan trajectory conversion utilities.")


if __name__ == "__main__":
    main()