import numpy as np
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D

# make a dataclass VehicleState that stores - time (float), position (WorldFrameCoordinate), velocity (Vector2D), and heading (float)
from dataclasses import dataclass


@dataclass
class VehicleState:
    position: WorldFrameCoordinate
    airspeed: float
    velocity_ground: Vector2D
    heading: float  # radians, 0 = East, pi/2 = North
    time: float  # seconds

    def __str__(self):
        return f"Position: ({self.position.x:.2f}, {self.position.y:.2f}), Heading: {np.degrees(self.heading):.2f} deg, Airspeed: {self.velocity_air.norm():.2f} m/s, Groundspeed: {self.velocity_ground.norm():.2f} m/s"

    def __repr__(self):
        return self.__str__()

    def get_airspeed_vector(self):
        return Vector2D(
            self.airspeed * np.cos(self.heading),
            self.airspeed * np.sin(self.heading),
        )

    def validate(self):
        assert isinstance(
            self.position, WorldFrameCoordinate
        ), "Position must be a WorldFrameCoordinate"
        assert isinstance(self.airspeed, float), "Airspeed must be a float"
        assert isinstance(
            self.velocity_ground, Vector2D
        ), "Velocity_ground must be a Vector2D"
        assert isinstance(self.heading, float), "Heading must be a float"
        assert (
            -np.pi <= self.heading <= np.pi
        ), "Heading must be in radians between -pi and pi"
        assert self.time > 0, "Time must be positive"


class VehicleStateEstimatorPassthrough:
    """For now, just a passthrough estimator that returns the input state as the estimated state."""

    def __init__(self, starting_state: VehicleState):
        self.state = starting_state

    def update(
        self,
        glider_position: WorldFrameCoordinate,
        true_airspeed: float,
        true_heading: float,
        dt: float,
    ):
        assert dt > 0, "Time step must be positive."
        # update the state based on the kinematic model
        position = WorldFrameCoordinate(glider_position.x, glider_position.y)

        # determine the ground velocity by differentiating the delta position over dt
        position_prev = self.state.position
        delta_position = position - position_prev
        velocity_ground = Vector2D(delta_position.x / dt, delta_position.y / dt)

        heading = true_heading
        self.state = VehicleState(position, true_airspeed, velocity_ground, heading)
        self.state.time += dt
        self.state.validate()

    def get_state(self):
        return self.state
