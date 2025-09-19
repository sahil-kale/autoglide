from enum import Enum
import numpy as np
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
        thermal_confidence_circle_threshold,
        glider_model_params,
        avg_thermal_strength_threshold_cruise_to_probe,
        min_probe_time_s,
        circling_confidence_abort_threshold,
        avg_thermal_strength_threshold_hysteresis=0.5,
    ):
        self.state = GuidanceState.CRUISE
        self.thermal_confidence_circle_threshold = thermal_confidence_circle_threshold
        self.avg_thermal_strength_threshold_cruise_to_probe = (
            avg_thermal_strength_threshold_cruise_to_probe
        )
        self.circling_confidence_abort_threshold = circling_confidence_abort_threshold
        self.avg_thermal_strength_threshold_hysteresis = (
            avg_thermal_strength_threshold_hysteresis
        )
        self.min_probe_time_s = min_probe_time_s
        self.last_state_change_time = 0.0

        self.cruise_control_law = CruiseControlLaw(
            cruise_speed=25.0,
            target_waypoint=None,
            origin_waypoint=None,
        )

        self.circling_control_law = CirclingControlLaw(
            lookahead_distance=20.0, glider_model_params=glider_model_params
        )

        self.probe_control_law = ProbeControlLaw(roll_angle_deg=60.0)

        self.state_to_control_law = {
            GuidanceState.CRUISE: self.cruise_control_law,
            GuidanceState.PROBE: self.probe_control_law,
            GuidanceState.CIRCLE: self.circling_control_law,
        }

    def get_state(self):
        return self.state

    def _can_transition(self, from_state, to_state, **kwargs):
        """Override or extend this method for custom transition logic."""
        # Default: allow all transitions
        return True

    def _cruise_to_probe(self, vehicle_state, thermal_estimate, current_time):
        return (
            thermal_estimate.get_average_sampled_thermal_strength()
            >= self.avg_thermal_strength_threshold_cruise_to_probe
        )

    def _probe_to_circle(self, vehicle_state, thermal_estimate, current_time):
        return thermal_estimate.confidence >= self.thermal_confidence_circle_threshold

    def _probe_to_cruise(self, vehicle_state, thermal_estimate, current_time):
        return (
            thermal_estimate.get_average_sampled_thermal_strength()
            <= (
                self.avg_thermal_strength_threshold_cruise_to_probe
                - self.avg_thermal_strength_threshold_hysteresis
            )
            and (current_time - self.last_state_change_time) >= self.min_probe_time_s
        )

    def _circle_to_cruise(self, vehicle_state, thermal_estimate, current_time):
        return thermal_estimate.confidence <= self.circling_confidence_abort_threshold

    def _transition_conditions(self):
        """
        Returns a dict mapping (from_state, to_state) -> condition function.
        Each function takes (vehicle_state, thermal_estimate, current_time) and returns bool.
        """
        return {
            (GuidanceState.CRUISE, GuidanceState.PROBE): self._cruise_to_probe,
            (GuidanceState.PROBE, GuidanceState.CIRCLE): self._probe_to_circle,
            (GuidanceState.PROBE, GuidanceState.CRUISE): self._probe_to_cruise,
            (GuidanceState.CIRCLE, GuidanceState.CRUISE): self._circle_to_cruise,
        }

    def _get_next_state(self, vehicle_state, thermal_estimate, current_time):
        transitions = self._transition_conditions()
        for (from_state, to_state), cond in transitions.items():
            if self.state == from_state and cond(
                vehicle_state, thermal_estimate, current_time
            ):
                return to_state
        return self.state

    def step(
        self,
        vehicle_state: VehicleState,
        thermal_estimate: ThermalEstimate,
        origin_wp: WorldFrameCoordinate,
        target_wp: WorldFrameCoordinate,
    ):
        current_time = vehicle_state.time
        self.cruise_control_law.origin_waypoint = origin_wp
        self.cruise_control_law.target_waypoint = target_wp

        prev_state = self.state
        next_state = self._get_next_state(vehicle_state, thermal_estimate, current_time)
        state_change = prev_state != next_state
        if state_change:
            self.last_state_change_time = current_time
            print(
                f"State changed from {prev_state} to {next_state} at time {current_time:.1f}s"
            )
        self.state = next_state
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
