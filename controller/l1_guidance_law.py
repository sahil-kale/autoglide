import numpy as np
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D


class L1GuidanceLaw:
    def __init__(self):
        self.L1_distance = 20.0  # Lookahead distance (m)
        self.g = 9.81  # Gravity (m/s^2)

    def compute_lateral_acceleration_for_waypoint(
        self,
        ground_speed_vector: Vector2D,
        current_location: WorldFrameCoordinate,
        lookahead_point: WorldFrameCoordinate,
    ):
        l1_vector = lookahead_point - current_location
        l1_distance = l1_vector.norm()
        eta = ground_speed_vector.signed_angle_between(l1_vector)
        ground_speed = ground_speed_vector.norm()
        a_s_cmd = 2 * ground_speed**2 / self.L1_distance * np.sin(eta)
        return a_s_cmd

    def compute_bank_angle_for_lateral_acceleration(self, a_s_cmd):
        return np.arctan2(a_s_cmd, self.g)
