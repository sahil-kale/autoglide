import numpy as np
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D, closest_point_to_segment
from controller.l1_guidance_law import L1GuidanceLaw
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate
from glider_model.model import GliderKinematicModelControl


class CruiseControlLaw:
    def __init__(
        self,
        cruise_speed,
        target_waypoint: WorldFrameCoordinate,
        origin_waypoint: WorldFrameCoordinate,
    ):
        self.cruise_speed = cruise_speed
        self.target_waypoint = target_waypoint
        self.origin_waypoint = origin_waypoint
        self.lookahead_distance = 30.0  # meters

    def compute_control(
        self, vehicle_state: VehicleState, thermal_estimate: ThermalEstimate, reset=True
    ):
        # compute the closest point along the path to the vehicle's current position
        vehicle_pos = vehicle_state.position
        closest_point = closest_point_to_segment(
            self.origin_waypoint, self.target_waypoint, vehicle_pos
        )
        path_vector = self.target_waypoint - self.origin_waypoint
        path_unit_vector = path_vector.normalize()
        lookahead_point = closest_point + path_unit_vector * self.lookahead_distance

        l1_guidance = L1GuidanceLaw()
        ground_speed_vector = vehicle_state.velocity_ground
        bank_angle_command = l1_guidance.get_L1_bank_angle(
            ground_speed_vector, vehicle_pos, lookahead_point
        )

        control = GliderKinematicModelControl(
            roll_angle=bank_angle_command, airspeed=self.cruise_speed
        )

        return control
