from dataclasses import dataclass
from enum import Enum, auto
from body_rate_controller.sys_id.trim_controller import (
    TrimConfig,
    GliderAttitudeTrimController,
)
from jsbsim_sandbox.sandbox_sim import JSBSim_Sandbox
from utils.pid import PIDConfig
import numpy as np


class PerturbationPhase(Enum):
    """Enumeration of the different phases in the perturbation profile."""

    INIT = auto()
    SINGLE_AXIS_STEP_PERTURBATION = auto()
    RANDOM_ALL_AXIS_PERTURBATION = auto()
    PERTURBATION_FINISHED = auto()


class PerturbationActuators(Enum):
    """Enumeration of the different actuators that can be perturbed."""

    AILERON = auto()
    ELEVATOR = auto()
    RUDDER = auto()


@dataclass
class PerturbationConfig:
    """
    Configuration for perturbations applied to the aircraft during trimming.

    Profile:

    # STEP 1: Single Axis Step Perturbation. Done for each actuator in [aileron, elevator, rudder].
    1) Step input in actuator to + perturbation_magnitude.
    2) Hold the perturbation for perturbation_duration_hold_phase_s seconds.
    3) Step input in actuator to - perturbation_magnitude for perturbation_duration_step_phase_s seconds.
    4) Trim the aircraft back to the original trim attitude.

    # STEP 2: All Axes Random Perturbation.
    1) For each actuator in [aileron, elevator, rudder], apply a random step input in
       [-perturbation_magnitude, +perturbation_magnitude].
    2) Hold the perturbation for perturbation_duration_random_phase_s seconds.
    3) Return all actuators to neutral (0.0).
    4) Repeat for num_random_perturbation_cycles.
    """

    elevator_perturbation_magnitude: float
    aileron_perturbation_magnitude: float
    rudder_perturbation_magnitude: float
    perturbation_duration_hold_phase_s: float
    perturbation_duration_step_phase_s: float
    perturbation_duration_neutral_phase_s: float
    perturbation_duration_random_phase_s: float
    num_random_perturbation_cycles: int


class Perturber:
    def __init__(self, sim: JSBSim_Sandbox, trim: GliderAttitudeTrimController) -> None:
        """
        Initialize the Perturber with a JSBSim sandbox and a GliderAttitudeTrimController.

        Args:
            sim (JSBSim_Sandbox): The JSBSim sandbox simulation instance.
            trim (GliderAttitudeTrimController): The trim controller instance for the particular trim sequence being perturbed.
            Note: There is an implicit assumption that the trim controller has already been run to trim before, and the sim
            is currently at the trimmed state.
        """

        self.sim = sim
        self.trim = trim
        self.reset()

    def reset(self) -> None:
        self.truth_state_history: list = []
        self.state_transition_time = self.sim.get_sim_time_s()
        self.actuator_phase_transition_time = self.sim.get_sim_time_s()
        self.state = PerturbationPhase.INIT
        self.actuator_phase = SingleAxisStepPerturbationPhase.ACTUATOR_INITIAL_STEP


if __name__ == "__main__":
    from jsbsim_sandbox.sandbox_sim import (
        JSBSimVehicleInitialCond,
        JSBSimSimParams,
        JSBSimVehicleConfig,
    )

    initial_cond = JSBSimVehicleInitialCond(
        h0_m=1000.0,
        vt0_mps=30.0,
        lat0_deg=0.0,
        lon0_deg=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
        psi0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=0.005)
    vehicle_config = JSBSimVehicleConfig(
        model_name="ask21",
        root_dir="jsbsim_sandbox/",
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)
    roll_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)
    pitch_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)
    yaw_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)

    trim_controller = GliderAttitudeTrimController(
        sim,
        dt=0.01,
        roll_gains=roll_gains,
        pitch_gains=pitch_gains,
        yaw_gains=yaw_gains,
        trim_config=TrimConfig(
            trim_angle_threshold_deg=5.0,
            max_d_body_rate_degps2=0.1,
        ),
    )

    trim_controller.run_until_trim(
        target_roll_rad=np.deg2rad(0),
        target_pitch_rad=np.deg2rad(0),
        target_sideslip_rad=np.deg2rad(0),
    )

    PerturbationConfig(
        elevator_perturbation_magnitude=0.1,
        aileron_perturbation_magnitude=0.1,
        rudder_perturbation_magnitude=0.1,
        perturbation_duration_hold_phase_s=2.0,
        perturbation_duration_step_phase_s=2.0,
        perturbation_duration_neutral_phase_s=2.0,
        perturbation_duration_random_phase_s=3.0,
        num_random_perturbation_cycles=3,
    )
    perturber = Perturber(
        sim,
        trim_controller,
        PerturbationConfig(
            elevator_perturbation_magnitude=0.1,
            aileron_perturbation_magnitude=0.1,
            rudder_perturbation_magnitude=0.1,
            perturbation_duration_hold_phase_s=2.0,
            perturbation_duration_step_phase_s=2.0,
            perturbation_duration_neutral_phase_s=2.0,
            perturbation_duration_random_phase_s=3.0,
            num_random_perturbation_cycles=3,
        ),
    )

    while perturber.state != PerturbationPhase.PERTURBATION_FINISHED:
        perturber.step()
