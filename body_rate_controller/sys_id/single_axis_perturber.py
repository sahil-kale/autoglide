from dataclasses import dataclass
from enum import Enum, auto
import click
import numpy as np


class SingleAxisPerturberType(Enum):
    """Enumeration of the different types of single axis perturbations."""

    STEP = auto()
    RANDOM = auto()
    CHIRP = auto()


@dataclass
class SingleAxesPerturberStepEvent:
    duration_s: float
    magnitude: float


@dataclass
class SingleAxisPerturberRandomEvent:
    duration_s: float
    magnitude: float


@dataclass
class SingleAxisPerturberChirpEvent:
    duration_s: float
    magnitude: float
    frequency_map: tuple[float, float]  # (start_freq, end_freq)
    theta: float = 0.0  # Phase offset


@dataclass
class SingleAxisPerturberEvent:
    event_type: SingleAxisPerturberType
    event_data: (
        SingleAxesPerturberStepEvent
        | SingleAxisPerturberRandomEvent
        | SingleAxisPerturberChirpEvent
    )


class SingleAxisPerturber:
    def __init__(self, events: list[SingleAxisPerturberEvent]) -> None:
        self.events = events
        self.current_event_index = 0
        self.event_start_time_s = 0.0
        self.active = False

    def start(self, current_time_s: float) -> None:
        self.active = True
        self.current_event_index = 0
        self.event_start_time_s = current_time_s
        self.event_elapsed_time_s = 0.0

    def step(self, current_time_s: float, dt_s: float) -> float:
        if not self.active or self.current_event_index >= len(self.events):
            raise RuntimeError(
                click.secho(
                    "Perturber is not active or all events have been completed.",
                    fg="red",
                    bold=True,
                    err=True,
                )
            )

        current_event = self.events[self.current_event_index]
        elapsed_time_s = current_time_s - self.event_start_time_s

        if elapsed_time_s >= current_event.event_data.duration_s:
            # Move to the next event
            self.current_event_index += 1
            if self.current_event_index < len(self.events):
                self.event_start_time_s = current_time_s
                self.event_elapsed_time_s = 0.0
                current_event = self.events[self.current_event_index]
            else:
                self.active = False  # All events completed
                return 0.0
        else:
            self.event_elapsed_time_s = elapsed_time_s

        return self.get_perturbation(current_event, dt_s)

    def get_perturbation(
        self, current_event: SingleAxisPerturberEvent, dt_s: float
    ) -> float:
        # Apply perturbation based on event type
        if current_event.event_type == SingleAxisPerturberType.STEP:
            return current_event.event_data.magnitude

        if current_event.event_type == SingleAxisPerturberType.RANDOM:
            assert (
                current_event.event_data.magnitude >= 0.0
            ), "Random perturbation magnitude must be non-negative."
            return np.random.uniform(
                -current_event.event_data.magnitude, current_event.event_data.magnitude
            )

        if current_event.event_type == SingleAxisPerturberType.CHIRP:
            if current_event.event_data.frequency_map is None:
                raise RuntimeError(
                    "Chirp event requires a frequency_map (start_freq, end_freq)."
                )
            start_freq, end_freq = current_event.event_data.frequency_map
            t = self.event_elapsed_time_s
            T = current_event.event_data.duration_s
            freq_idx = t / T if T > 0 else 0.0  # Normalized time index
            instantaneous_freq = start_freq + (end_freq - start_freq) * freq_idx
            current_event.event_data.theta += 2 * np.pi * instantaneous_freq * dt_s
            return current_event.event_data.magnitude * np.sin(
                current_event.event_data.theta
            )

        msg = click.secho(
            f"Unknown perturber event type: {current_event.event_type}",
            fg="red",
            bold=True,
            err=True,
        )
        raise RuntimeError(msg)

    def is_active(self) -> bool:
        return self.active


if __name__ == "__main__":
    from jsbsim_sandbox.sandbox_sim import (
        JSBSim_Sandbox,
        JSBSimVehicleConfig,
        JSBSimVehicleInitialCond,
        JSBSimSimParams,
    )
    from vehicle_interface.vehicle_interface import ControlCommands
    from utils.pid import PIDConfig
    from body_rate_controller.sys_id.trim_controller import (
        GliderAttitudeTrimController,
        TrimConfig,
        TrimTarget,
    )

    # Initialize the simulation environment
    initial_cond = JSBSimVehicleInitialCond(
        h0_m=1000.0,
        vt0_mps=30.0,
        lat0_deg=0.0,
        lon0_deg=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
        psi0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=0.01)
    vehicle_config = JSBSimVehicleConfig(
        model_name="ask21",
        root_dir="jsbsim_sandbox/",
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)

    # Configure the trim controller
    roll_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)
    pitch_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)
    yaw_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)

    trim_controller = GliderAttitudeTrimController(
        sim,
        dt=0.01,
        roll_gains=roll_gains,
        pitch_gains=pitch_gains,
        yaw_gains=yaw_gains,
        trim_config=TrimConfig(
            trim_angle_threshold_deg=5.0,
            max_d_body_rate_degps2=0.1,
            trim_persistence_threshold_s=0.5,
        ),
    )

    # Trim the plane to 0 degrees roll, pitch, and sideslip
    trim_target = TrimTarget(
        roll_rad=0.0,
        pitch_rad=0.0,
        sideslip_rad=0.0,
    )

    trim_controller.run_until_trim(
        trim_target=trim_target,
        max_steps=8000,
        raise_on_fail=True,
    )

    # Define a chirp perturbation event
    chirp_event = SingleAxisPerturberEvent(
        event_type=SingleAxisPerturberType.CHIRP,
        event_data=SingleAxisPerturberChirpEvent(
            duration_s=100.0,
            magnitude=1.0,
            frequency_map=(0.1, 2.0),
        ),
    )

    perturber = SingleAxisPerturber(events=[chirp_event])
    perturber.start(current_time_s=0.0)

    # Apply the chirp perturbation
    control_commands = ControlCommands(0.0, 0.0, 0.0, 0.0)
    while sim.get_sim_time_s() < chirp_event.event_data.duration_s:
        perturbation = perturber.step(
            current_time_s=sim.get_sim_time_s(), dt_s=sim_params.dt_s
        )

        # Apply perturbation to the aileron as an example
        control_commands.elevator_deflection_norm = perturbation

        truth_data = sim.step(control_commands)

        # Log the simulation state
        print(
            f"Time: {sim.get_sim_time_s():.2f}s, Perturbation: {perturbation:.3f}, "
            f"Roll: {np.rad2deg(truth_data.attitude.get_euler()[0]):.2f} deg"
        )

    from jsbsim_sandbox.vehicle_state_visualizer import animate_sim

    animate_sim(sim.sim_truth_state_history, interval_ms=0.005, frame_step=10)
