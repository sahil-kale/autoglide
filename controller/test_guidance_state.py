from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate


def test_initial_state():
    gsm = GuidanceStateMachine()
    assert gsm.get_state() == GuidanceState.CRUISE


def test_cruise_to_probe_transition():
    gsm = GuidanceStateMachine(thermal_confidence_threshold=0.3)
    vehicle_state = VehicleState(
        position=None,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=None,  # Not used in current logic
        heading=0.0,
    )
    thermal_estimate = ThermalEstimate(W0=5, Rth=50, x_th=10, y_th=10, confidence=0.2)
    gsm.step(vehicle_state, thermal_estimate)
    assert (
        gsm.get_state() == GuidanceState.CRUISE
    )  # Should remain in CRUISE due to low confidence
    thermal_estimate.confidence = 0.4
    gsm.step(vehicle_state, thermal_estimate)
    assert gsm.get_state() == GuidanceState.PROBE  # Should transition to PROBE
