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

        # Roll and Yaw use the same generic profile.
        generic_axis_perturbation_magnitude = 0.4
        self.roll_perturbation_config = [
            # +Step
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=generic_axis_perturbation_magnitude,
                duration_s=0.5,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=0.0,
                duration_s=0.5,
            ),
            # -Step
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=-generic_axis_perturbation_magnitude,
                duration_s=0.5,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=generic_axis_perturbation_magnitude,
                duration_s=1.0,
            ),
        ]

        elevator_perturbation_magnitude = 0.4
        self.pitch_perturbation_config = [
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=-elevator_perturbation_magnitude,
                duration_s=0.25,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=0.0,
                duration_s=0.25,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=elevator_perturbation_magnitude,
                duration_s=0.5,
            ),
            SingleAxisPerturberEvent(
                event_type=SingleAxisPerturberType.STEP,
                magnitude=elevator_perturbation_magnitude * 0.5,
                duration_s=0.5,
            ),
        ]

        # Yaw uses the same generic profile as roll.
        self.yaw_perturbation_config = list(self.roll_perturbation_config)

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
        match axis:
            case BodyRateModelPerturberAxes.ROLL:
                return SingleAxisPerturber(events=self.roll_perturbation_config)
            case BodyRateModelPerturberAxes.PITCH:
                return SingleAxisPerturber(events=self.pitch_perturbation_config)
            case BodyRateModelPerturberAxes.YAW:
                return SingleAxisPerturber(events=self.yaw_perturbation_config)
            case _:
                raise ValueError(
                    click.secho(
                        f"Unknown perturber axis: {axis}",
                        fg="red",
                        bold=True,
                        err=True,
                    )
                )

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
