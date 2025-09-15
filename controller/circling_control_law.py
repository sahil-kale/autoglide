import numpy as np
from utils.location import WorldFrameCoordinate
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import (
    ThermalEstimate,
    ReducedOrderGaussianThermalModel,
)
from glider_model.model import GliderKinematicModelControl, GliderModelParams
from controller.l1_guidance_law import L1GuidanceLaw
from scipy.optimize import minimize


class CirclingControlLaw:
    def __init__(self, lookahead_distance, glider_model_params: GliderModelParams):
        self.lookahead_distance = lookahead_distance
        self.glider_model_params = glider_model_params

    def find_optimal_radius_and_speed(
        self, vehicle_state, thermal_estimate: ThermalEstimate
    ):
        initial_guess = [thermal_estimate.Rth, vehicle_state.airspeed]

        def objective(params):
            R_to_fly, V = params
            gaussian_model = ReducedOrderGaussianThermalModel(thermal_estimate)
            location_at_R_to_fly = WorldFrameCoordinate(
                thermal_estimate.est_core.x + R_to_fly, thermal_estimate.est_core.y
            )
            w_at_R_to_fly = gaussian_model.thermal_model_estimate_updraft(
                location_at_R_to_fly
            )
            phi = np.arctan2(V**2, 9.81 * R_to_fly)
            sink_rate = self.glider_model_params.get_sink_rate(V, phi)
            net_climb_rate = w_at_R_to_fly - sink_rate
            return -net_climb_rate

            # note this is a maximization problem, so we minimize the negative of the objective

        bounds = [(1.0, None), (self.glider_model_params.s_min - 10, None)]
        result = minimize(objective, initial_guess, bounds=bounds)
        if result.success:
            optimal_R, optimal_V = result.x
        else:
            optimal_R = thermal_estimate.Rth * 0.35
            optimal_V = self.glider_model_params.V_star  # min sink
            print("Circling Control Law failed, using default values.")

        return optimal_R, optimal_V

    def compute_control(
        self, vehicle_state: VehicleState, thermal_estimate: ThermalEstimate, reset
    ):
        # 0) Find the optimal radius and speed to circle at
        optimal_R, optimal_V = self.find_optimal_radius_and_speed(
            vehicle_state, thermal_estimate
        )

        # 1) Find the vector from the thermal core to the aircraft position
        vector_to_aircraft = vehicle_state.position - thermal_estimate.est_core

        # 2) Determine the absolute angle of this vector (atan2)
        angle_to_aircraft = np.arctan2(
            vector_to_aircraft.y, vector_to_aircraft.x
        )  # global angle in radians

        # 3) Add the lookahead distance to this angle to get the target point angle (theta += lookahead/radius)
        target_point_angle = angle_to_aircraft + (self.lookahead_distance / optimal_R)

        # 4) Compute the target point coordinates using the thermal core location and the radius
        target_point_x = thermal_estimate.est_core.x + optimal_R * np.cos(
            target_point_angle
        )
        target_point_y = thermal_estimate.est_core.y + optimal_R * np.sin(
            target_point_angle
        )
        target_point = WorldFrameCoordinate(target_point_x, target_point_y)

        # 5) Use L1 guidance to compute the bank angle to this target point
        l1_guidance = L1GuidanceLaw()
        ground_speed_vector = vehicle_state.velocity_ground
        bank_angle_command = l1_guidance.get_L1_bank_angle(
            ground_speed_vector, vehicle_state.position, target_point
        )

        control = GliderKinematicModelControl(
            roll_angle=bank_angle_command, airspeed=optimal_V
        )

        return control
