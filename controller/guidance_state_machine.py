from enum import Enum
from vehicle_state_estimator.vehicle_state_estimator import VehicleState


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
    def __init__(self):
        self.state = GuidanceState.CRUISE

    def get_state(self):
        return self.state

    def step(self, vehicle_state: VehicleState):
        pass
