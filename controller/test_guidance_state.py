from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState
from glider_model.model import GliderModelParams
from vehicle_state_estimator.vehicle_state_estimator import VehicleState
from thermal_estimator.thermal_estimator import ThermalEstimate
from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D

DEFAULT_ORIGIN_WP = WorldFrameCoordinate(0.0, 0.0)
DEFAULT_TARGET_WP = WorldFrameCoordinate(1000.0, 0.0)
DEFAULT_ZERO_VELOCITY_VECTOR = Vector2D(0.0, 0.0)
DEFAULT_GLIDER_MODEL_PARAMS = GliderModelParams(
    V_star=15.0,
    V_stall=10.0,
    s_min=0.5,
    k_v=0.02,
    alpha_n=2.0,
    initial_altitude=1000.0,
    vel_tau=1.0,
    roll_tau=1.0,
)

DEFAULT_THERMAL_ESTIMATE = ThermalEstimate(
    W0=5,
    Rth=50,
    est_core=DEFAULT_ORIGIN_WP,
    confidence=0.0,
    average_actual_thermal_strength=0.0,
)


def test_initial_state():
    gsm = GuidanceStateMachine(
        thermal_confidence_circle_threshold=0.5,
        glider_model_params=DEFAULT_GLIDER_MODEL_PARAMS,
        avg_thermal_strength_threshold_cruise_to_probe=2.0,
        avg_thermal_strength_threshold_hysteresis=1.0,
    )
    assert gsm.get_state() == GuidanceState.CRUISE


def test_cruise_to_probe_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_circle_threshold=0.5,
        glider_model_params=DEFAULT_GLIDER_MODEL_PARAMS,
        avg_thermal_strength_threshold_cruise_to_probe=2.0,
        avg_thermal_strength_threshold_hysteresis=1.0,
    )
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )

    thermal_estimate = DEFAULT_THERMAL_ESTIMATE

    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.CRUISE

    thermal_estimate.average_actual_thermal_strength = (
        gsm.avg_thermal_strength_threshold_cruise_to_probe
    )
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.PROBE  # Should transition to PROBE


def test_probe_to_circle_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_circle_threshold=0.5,
        glider_model_params=DEFAULT_GLIDER_MODEL_PARAMS,
        avg_thermal_strength_threshold_cruise_to_probe=2.0,
        avg_thermal_strength_threshold_hysteresis=1.0,
    )
    gsm.state = GuidanceState.PROBE
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )
    thermal_estimate = DEFAULT_THERMAL_ESTIMATE
    thermal_estimate.confidence = 0.4  # Below threshold
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert (
        gsm.get_state() == GuidanceState.PROBE
    )  # Should remain in PROBE due to low confidence
    thermal_estimate.confidence = 0.6
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.CIRCLE  # Should transition to CIRCLE


def test_probe_to_cruise_transition():
    gsm = GuidanceStateMachine(
        thermal_confidence_circle_threshold=0.5,
        glider_model_params=DEFAULT_GLIDER_MODEL_PARAMS,
        avg_thermal_strength_threshold_cruise_to_probe=2.0,
        avg_thermal_strength_threshold_hysteresis=1.0,
    )
    gsm.state = GuidanceState.PROBE
    vehicle_state = VehicleState(
        position=DEFAULT_ORIGIN_WP,  # Not used in current logic
        airspeed=10.0,
        velocity_ground=DEFAULT_ZERO_VELOCITY_VECTOR,  # Not used in current logic
        heading=0.0,
    )
    # Stay in PROBE (above hysteresis threshold)
    thermal_estimate = DEFAULT_THERMAL_ESTIMATE
    thermal_estimate.confidence = 0.4
    thermal_estimate.average_actual_thermal_strength = 2.0
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert (
        gsm.get_state() == GuidanceState.PROBE
    )  # Should remain in PROBE due to sufficient avg strength

    thermal_estimate.average_actual_thermal_strength = 0.5
    thermal_estimate.confidence = 0.4
    gsm.step(vehicle_state, thermal_estimate, DEFAULT_ORIGIN_WP, DEFAULT_TARGET_WP)
    assert gsm.get_state() == GuidanceState.CRUISE  # Should transition back to CRUISE
