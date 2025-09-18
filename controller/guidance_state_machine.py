from enum import Enum
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate
from controller.cruise_control_law import CruiseControlLaw
from controller.probe_control_law import ProbeControlLaw
from controller.circling_control_law import CirclingControlLaw
from utils.location import WorldFrameCoordinate


class GuidanceState(Enum):
    CRUISE = (1, "Cruise")
    PROBE = (2, "Probe")
    CIRCLE = (3, "Circle")

    def __init__(self, num, string):
        self.num = num
        self.string = string

    def __str__(self):
        return self.string


class GuidanceStateMachine:
    def __init__(
        self,
        thermal_confidence_probe_threshold,
        thermal_confidence_circle_threshold,
        glider_model_params,
    ):
        self.state = GuidanceState.CRUISE
        self.thermal_confidence_probe_threshold = thermal_confidence_probe_threshold
        self.thermal_confidence_circle_threshold = thermal_confidence_circle_threshold

        self.cruise_control_law = CruiseControlLaw(
            cruise_speed=25.0,
            target_waypoint=None,
            origin_waypoint=None,
        )

        self.circling_control_law = CirclingControlLaw(
            lookahead_distance=20.0, glider_model_params=glider_model_params
        )

        self.probe_control_law = ProbeControlLaw(roll_angle_deg=45.0)

        self.state_to_control_law = {
            GuidanceState.CRUISE: self.cruise_control_law,
            GuidanceState.PROBE: self.probe_control_law,
            GuidanceState.CIRCLE: self.circling_control_law,
        }

    def get_state(self):
        return self.state

    def step(
        self,
        vehicle_state: VehicleState,
        thermal_estimate: ThermalEstimate,
        origin_wp: WorldFrameCoordinate,
        target_wp: WorldFrameCoordinate,
    ):
        self.cruise_control_law.origin_waypoint = origin_wp
        self.cruise_control_law.target_waypoint = target_wp

        prev_state = self.state
        if prev_state == GuidanceState.CRUISE:
            # TODO: Consider adding minimum W0 threshold to avoid weak thermals.
            # Implement at same time as MacCready logic since that will affect climb decisions.
            if thermal_estimate.confidence >= self.thermal_confidence_probe_threshold:
                self.state = GuidanceState.PROBE
        elif prev_state == GuidanceState.PROBE:
            if thermal_estimate.confidence >= self.thermal_confidence_circle_threshold:
                self.state = GuidanceState.CIRCLE
            elif thermal_estimate.confidence < self.thermal_confidence_probe_threshold:
                self.state = GuidanceState.CRUISE
        elif prev_state == GuidanceState.CIRCLE:
            # TODO: add logic to exit circle state if
            # 1) Thermal confidence drops below a threshold
            # 2) Thermal strength drops below a threshold
            # 3) High level planner issues command to exit (ex: enough altitude gained, or cloud base reached)
            pass

        state_change = prev_state != self.state
        control_law = self.state_to_control_law[self.state]

        if control_law is not None:
            control = control_law.compute_control(
                vehicle_state, thermal_estimate, reset=state_change
            )
        else:
            raise NotImplementedError(
                f"Control law for state {self.state} not implemented."
            )

        return control
