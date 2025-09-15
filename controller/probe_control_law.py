from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate
from glider_model.model import GliderKinematicModelControl
from utils.vector import Vector2D
import numpy as np


class ProbeControlLaw:
    def __init__(self, roll_angle_deg):
        self.roll_angle = np.deg2rad(roll_angle_deg)  # convert to radians

    def compute_control(
        self, vehicle_state: VehicleState, thermal_estimate: ThermalEstimate, reset
    ):
        roll_angle_command = self.roll_angle
        # determine the angle to the estimated thermal core
        # and adjust the roll angle to point towards it
        # (this is a simplified placeholder logic)

        vector_to_thermal = thermal_estimate.est_core - vehicle_state.position
        angle_to_thermal = vehicle_state.position.signed_angle_between(
            vector_to_thermal
        )
        if angle_to_thermal < 0:
            roll_angle_command = -self.roll_angle  # turn right

        # for now, just return a fixed bank angle and the previous airspeed
        control = GliderKinematicModelControl(
            roll_angle=roll_angle_command, airspeed=vehicle_state.airspeed
        )
        return control
