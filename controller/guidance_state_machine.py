from enum import Enum
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate


class GuidanceState(Enum):
    CRUISE = (1, "cruise")
    PROBE = (2, "probe")
    CIRCLE = (3, "circle")

    def __init__(self, num, string):
        self.num = num
        self.string = string

    def __str__(self):
        return self.string


class GuidanceStateMachine:
    def __init__(
        self, thermal_confidence_probe_threshold, thermal_confidence_circle_threshold
    ):
        self.state = GuidanceState.CRUISE
        self.thermal_confidence_probe_threshold = thermal_confidence_probe_threshold
        self.thermal_confidence_circle_threshold = thermal_confidence_circle_threshold

    def get_state(self):
        return self.state

    def step(self, vehicle_state: VehicleState, thermal_estimate: ThermalEstimate):
        if self.state == GuidanceState.CRUISE:
            # TODO: Consider adding minimum W0 threshold to avoid weak thermals.
            # Implement at same time as MacCready logic since that will affect climb decisions.
            if thermal_estimate.confidence >= self.thermal_confidence_probe_threshold:
                self.state = GuidanceState.PROBE
        elif self.state == GuidanceState.PROBE:
            if thermal_estimate.confidence >= self.thermal_confidence_circle_threshold:
                self.state = GuidanceState.CIRCLE
            elif thermal_estimate.confidence < self.thermal_confidence_probe_threshold:
                self.state = GuidanceState.CRUISE
        elif self.state == GuidanceState.CIRCLE:
            # TODO: add logic to exit circle state if
            # 1) Thermal confidence drops below a threshold
            # 2) Thermal strength drops below a threshold
            # 3) High level planner issues command to exit (ex: enough altitude gained, or cloud base reached)
            pass
