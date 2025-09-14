import numpy as np
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D
from vehicle_state_estimator import VehicleState, VehicleStateEstimatorPassthrough


def test_ground_vector_computation():
    # Initial state
    initial_position = WorldFrameCoordinate(0.0, 0.0)
    initial_airspeed = 10.0
    initial_velocity_ground = Vector2D(0.0, 0.0)
    initial_heading = 0.0
    state = VehicleState(
        initial_position, initial_airspeed, initial_velocity_ground, initial_heading
    )
    estimator = VehicleStateEstimatorPassthrough(state)

    # Simulate movement: move east by 10 meters in 1 second
    new_position = WorldFrameCoordinate(10.0, 0.0)
    true_airspeed = 10.0
    true_heading = 0.0
    dt = 1.0
    estimator.update(new_position, true_airspeed, true_heading, dt)
    updated_state = estimator.get_state()

    # Ground velocity should be (10, 0)
    assert np.isclose(updated_state.velocity_ground.x, 10.0)
    assert np.isclose(updated_state.velocity_ground.y, 0.0)
    assert np.isclose(updated_state.velocity_ground.norm(), 10.0)
    print("Test passed: ground vector computed correctly.")


if __name__ == "__main__":
    test_ground_vector_computation()
