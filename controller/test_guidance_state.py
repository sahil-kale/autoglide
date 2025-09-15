from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D

DEFAULT_ORIGIN_WP = WorldFrameCoordinate(0.0, 0.0)
DEFAULT_TARGET_WP = WorldFrameCoordinate(1000.0, 0.0)
DEFAULT_ZERO_VELOCITY_VECTOR = Vector2D(0.0, 0.0)


def test_initial_state():
    gsm = GuidanceStateMachine(
        thermal_confidence_probe_threshold=0.3, thermal_confidence_circle_threshold=0.5
    )
    assert gsm.get_state() == GuidanceState.CRUISE


def test_cruise_to_probe_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_probe_threshold=0.3, thermal_confidence_circle_threshold=0.5
    )
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )

    thermal_estimate = ThermalEstimate(
        W0=5, Rth=50, est_core=DEFAULT_ORIGIN_WP, confidence=0.2
    )
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert (
        gsm.get_state() == GuidanceState.CRUISE
    )  # Should remain in CRUISE due to low confidence
    thermal_estimate.confidence = 0.4
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.PROBE  # Should transition to PROBE


def test_probe_to_circle_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_probe_threshold=0.3, thermal_confidence_circle_threshold=0.5
    )
    gsm.state = GuidanceState.PROBE
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )
    thermal_estimate = ThermalEstimate(
        W0=5, Rth=50, est_core=DEFAULT_ORIGIN_WP, confidence=0.4
    )
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert (
        gsm.get_state() == GuidanceState.PROBE
    )  # Should remain in PROBE due to low confidence
    thermal_estimate.confidence = 0.6
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.CIRCLE  # Should transition to CIRCLE


def test_probe_to_cruise_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_probe_threshold=0.3, thermal_confidence_circle_threshold=0.5
    )
    gsm.state = GuidanceState.PROBE
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )
    thermal_estimate = ThermalEstimate(
        W0=5, Rth=50, est_core=DEFAULT_ORIGIN_WP, confidence=0.4
    )
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert (
        gsm.get_state() == GuidanceState.PROBE
    )  # Should remain in PROBE due to sufficient confidence
    thermal_estimate.confidence = 0.2
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.CRUISE  # Should transition back to CRUISE
