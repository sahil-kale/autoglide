from dataclasses import dataclass
from enum import Enum, auto
from body_rate_controller.sys_id.trim_controller import (
    TrimConfig,
    GliderAttitudeTrimController,
)
from jsbsim_sandbox.sandbox_sim import JSBSim_Sandbox
from utils.pid import PIDConfig


class SingleAxisStepPerturbationPhase(Enum):
    """Enumeration of the different phases in the perturbation profile."""

    ACTUATOR_INITIAL_STEP = auto()
    ACTUATOR_HOLD_POSITIVE = auto()
    ACTUATOR_HOLD_NEGATIVE = auto()
    ACTUATOR_PERTURBATION_FINISHED = auto()


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
        self.actuator_under_test: PerturbationActuators.AILERON

    def step(self) -> None:
        state_before_transition = self.state
        match self.state:
            case PerturbationPhase.INIT:
                self.state = PerturbationPhase.SINGLE_AXIS_STEP_PERTURBATION
                self.actuator_phase_transition_time = self.sim.get_sim_time_s()
            case PerturbationPhase.SINGLE_AXIS_STEP_PERTURBATION:
                if self.run_actuator_step_perturbation():
                    self.state = PerturbationPhase.RANDOM_ALL_AXIS_PERTURBATION
                    self.state_transition_time = self.sim.get_sim_time_s()
            case PerturbationPhase.RANDOM_ALL_AXIS_PERTURBATION:
                pass
            case PerturbationPhase.PERTURBATION_FINISHED:
                pass

        if state_before_transition != self.state:
            self.state_transition_time = self.sim.get_sim_time_s()

    def run_actuator_step_perturbation(self) -> bool:
        actuator_phase_before_transition = self.actuator_phase
        match self.actuator_phase:
            case SingleAxisStepPerturbationPhase.ACTUATOR_INITIAL_STEP:
                pass
            case SingleAxisStepPerturbationPhase.ACTUATOR_HOLD_POSITIVE:
                pass
            case SingleAxisStepPerturbationPhase.ACTUATOR_HOLD_NEGATIVE:
                pass
            case SingleAxisStepPerturbationPhase.ACTUATOR_PERTURBATION_FINISHED:
                pass

        if actuator_phase_before_transition != self.actuator_phase:
            self.actuator_phase_transition_time = self.sim.get_sim_time_s()

        return False
