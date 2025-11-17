import click
from dataclasses import dataclass
from enum import Enum, auto
from body_rate_controller.sys_id.trim_controller import (
    TrimConfig,
    GliderAttitudeTrimController,
    TrimTarget,
)
from jsbsim_sandbox.sandbox_sim import JSBSim_Sandbox
from utils.pid import PIDConfig
import numpy as np
from body_rate_controller.sys_id.single_axis_perturber import (
    SingleAxisPerturber,
    SingleAxisPerturberEvent,
    SingleAxisPerturberType,
)
from vehicle_interface.vehicle_interface import ControlCommands


class BodyRateModelPerturberAxes(Enum):
    ROLL = auto()
    PITCH = auto()
    YAW = auto()


class BodyRateModelPerturber:
    def __init__(
        self,
        sim: JSBSim_Sandbox,
        trim: GliderAttitudeTrimController,
        trim_target: TrimTarget,
    ):
        self.sim = sim
        self.trim = trim
        self.trim_target = trim_target

        # Hardcoded profile for now. Can be made configurable later.
        generic_axis_perturbation_magnitude = 0.1
        self.generic_axis_perturbation_config = [
            # +Step
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=generic_axis_perturbation_magnitude,
                duration_s=0.25,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=0.0,
                duration_s=0.25,
            ),
            # -Step
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=-generic_axis_perturbation_magnitude,
                duration_s=0.25,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=0.0,
                duration_s=0.25,
            ),
            # Random Binary/Uniform
            # SingleAxisPerturberEvent(
            #     event_type=SingleAxisPerturberType.RANDOM,
            #     magnitude=generic_axis_perturbation_magnitude,
            #     duration_s=2.0,
            # )
        ]

    def run(self):
        for axis in BodyRateModelPerturberAxes:
            perturber = self._get_perturber_for_axis(axis)
            click.secho(
                f"Starting perturbation sequence for axis: {axis.name}", fg="blue"
            )
            perturber.start(current_time_s=self.sim.get_sim_time_s())

            while perturber.active:
                perturbation = perturber.step(current_time_s=self.sim.get_sim_time_s())
                control_commands = self._get_control_command_from_perturbation(
                    perturbation, axis
                )
                self.sim.step(control_commands=control_commands)

            click.secho(
                f"Completed perturbation sequence for axis: {axis.name}", fg="green"
            )

            self.trim.reset()
            self.trim.run_until_trim(
                trim_target=self.trim_target,
            )
        click.secho("Completed all axis perturbation sequences.", fg="green", bold=True)

    def _get_perturber_for_axis(
        self, axis: BodyRateModelPerturberAxes
    ) -> SingleAxisPerturber:
        return SingleAxisPerturber(events=self.generic_axis_perturbation_config)

    def _get_control_command_from_perturbation(
        self, perturbation: float, axis: BodyRateModelPerturberAxes
    ) -> ControlCommands:
        control_commands = self.trim.sim_truth_state_history[-1].control_commands
        match axis:
            case BodyRateModelPerturberAxes.PITCH:
                control_commands.elevator_deflection_norm += perturbation
            case BodyRateModelPerturberAxes.ROLL:
                control_commands.aileron_deflection_norm += perturbation
            case BodyRateModelPerturberAxes.YAW:
                control_commands.rudder_deflection_norm += perturbation

            case _:
                raise ValueError(
                    click.secho(
                        f"Unknown perturber axis: {axis}",
                        fg="red",
                        bold=True,
                        err=True,
                    )
                )

        return control_commands


if __name__ == "__main__":
    from jsbsim_sandbox.sandbox_sim import (
        JSBSimVehicleConfig,
        JSBSimVehicleInitialCond,
        JSBSimSimParams,
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
            trim_persistence_threshold_s=0.1,
        ),
    )

    trim_target = TrimTarget(
        roll_rad=np.deg2rad(0),
        pitch_rad=np.deg2rad(0),
        sideslip_rad=np.deg2rad(0),
    )

    trim_controller.run_until_trim(
        trim_target=trim_target,
    )

    perturber = BodyRateModelPerturber(sim, trim_controller, trim_target)
    perturber.run()

    from jsbsim_sandbox.vehicle_state_visualizer import animate_sim

    animate_sim(sim.sim_truth_state_history, interval_ms=0.005, frame_step=10)
