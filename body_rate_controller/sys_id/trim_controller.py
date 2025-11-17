import numpy as np
import click
from dataclasses import dataclass
from typing import Optional

from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
)
from vehicle_interface.vehicle_interface import ControlCommands, SimTruthState
from utils.pid import PIDController, PIDConfig


@dataclass
class TrimConfig:
    trim_angle_threshold_deg: float
    max_d_body_rate_degps2: float
    trim_persistence_threshold_s: float


class GliderAttitudeTrimController:
    def __init__(
        self,
        sim: JSBSim_Sandbox,
        dt: float,
        roll_gains: PIDConfig,
        pitch_gains: PIDConfig,
        yaw_gains: PIDConfig,
        trim_config: TrimConfig | None = None,
    ) -> None:
        self.sim = sim
        self.dt = dt
        self.cfg = trim_config
        self.roll_controller = PIDController(roll_gains, dt)
        self.pitch_controller = PIDController(pitch_gains, dt)
        self.sideslip_controller = PIDController(yaw_gains, dt)

        self.reset()

    def reset(self):
        for controller in [
            self.roll_controller,
            self.pitch_controller,
            self.sideslip_controller,
        ]:
            controller.reset()

        self.sim_truth_state_history: list[SimTruthState] = []
        self.trim_angle_threshold_rad = np.deg2rad(self.cfg.trim_angle_threshold_deg)
        self.max_d_body_rate_radps2 = np.deg2rad(self.cfg.max_d_body_rate_degps2)
        self.in_trim_persistence_threshold_s = self.cfg.trim_persistence_threshold_s

        self.in_trim_persistence_counter_s = 0.0

    def step_trim_to_attitude(
        self,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float,
        truth_data: SimTruthState,
    ) -> ControlCommands:
        roll_rad, pitch_rad, _ = truth_data.attitude.get_euler()
        current_sideslip_rad = truth_data.sideslip_rad

        aileron_cmd = self.roll_controller.step(target_roll_rad, roll_rad)
        elevator_cmd = -self.pitch_controller.step(target_pitch_rad, pitch_rad)
        rudder_cmd = self.sideslip_controller.step(
            target_sideslip_rad, current_sideslip_rad
        )

        self.sim_truth_state_history.append(truth_data)

        return ControlCommands(
            aileron_deflection_norm=aileron_cmd,
            elevator_deflection_norm=elevator_cmd,
            rudder_deflection_norm=rudder_cmd,
            spoiler_deflection=0.0,
        )

    def run_until_trim(
        self,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float = 0.0,
        max_steps: int = 4000,
        raise_on_fail: bool = True,
    ) -> bool:
        click.secho(
            f"Starting trim to Roll: {np.rad2deg(target_roll_rad):.2f} deg, "
            f"Pitch: {np.rad2deg(target_pitch_rad):.2f} deg, "
            f"Sideslip: {np.rad2deg(target_sideslip_rad):.2f} deg",
            fg="blue",
        )

        control_commands = ControlCommands(0.0, 0.0, 0.0, 0.0)
        previous_body_rates: Optional[np.ndarray] = None
        d_body_rates_radps2 = np.zeros(3)

        for step in range(max_steps):
            truth_data = self.sim.step(control_commands)

            control_commands = self.step_trim_to_attitude(
                target_roll_rad=target_roll_rad,
                target_pitch_rad=target_pitch_rad,
                target_sideslip_rad=target_sideslip_rad,
                truth_data=truth_data,
            )

            body_rates = self._compute_body_rates(truth_data)

            if previous_body_rates is not None:
                d_body_rates_radps2 = self._compute_body_rate_derivative(
                    body_rates, previous_body_rates
                )

                if self._update_trim_persistence(
                    truth_data,
                    target_roll_rad=target_roll_rad,
                    target_pitch_rad=target_pitch_rad,
                    target_sideslip_rad=target_sideslip_rad,
                    d_body_rates_radps2=d_body_rates_radps2,
                ):
                    self._log_trim_success(truth_data, d_body_rates_radps2, step)
                    return True

            previous_body_rates = body_rates

        self._log_trim_failure(truth_data, d_body_rates_radps2)

        if raise_on_fail:
            raise RuntimeError("Failed to achieve trim.")

        return False

    def _compute_body_rates(self, truth_data: SimTruthState) -> np.ndarray:
        return np.array(
            [truth_data.p_radps, truth_data.q_radps, truth_data.r_radps],
            dtype=float,
        )

    def _compute_body_rate_derivative(
        self,
        body_rates: np.ndarray,
        previous_body_rates: np.ndarray,
    ) -> np.ndarray:
        return np.abs(body_rates - previous_body_rates) / self.dt

    def _compute_attitude_error(
        self,
        truth_data: SimTruthState,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float,
    ) -> np.ndarray:
        roll_rad, pitch_rad, _ = truth_data.attitude.get_euler()
        sideslip_rad = truth_data.sideslip_rad

        return np.array(
            [
                np.abs(target_roll_rad - roll_rad),
                np.abs(target_pitch_rad - pitch_rad),
                np.abs(target_sideslip_rad - sideslip_rad),
            ],
            dtype=float,
        )

    def _update_trim_persistence(
        self,
        truth_data: SimTruthState,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float,
        d_body_rates_radps2: np.ndarray,
    ) -> bool:
        accel_ok = np.all(d_body_rates_radps2 < self.max_d_body_rate_radps2)
        attitude_error = self._compute_attitude_error(
            truth_data,
            target_roll_rad=target_roll_rad,
            target_pitch_rad=target_pitch_rad,
            target_sideslip_rad=target_sideslip_rad,
        )
        attitude_ok = np.all(attitude_error < self.trim_angle_threshold_rad)

        if accel_ok and attitude_ok:
            self.in_trim_persistence_counter_s += self.dt
        else:
            self.in_trim_persistence_counter_s = 0.0

        return self.in_trim_persistence_counter_s >= (
            self.in_trim_persistence_threshold_s
        )

    def _format_trim_debug_info(
        self,
        truth_data: SimTruthState,
        d_body_rates_radps2: np.ndarray,
    ) -> str:
        roll_rad, pitch_rad, _ = truth_data.attitude.get_euler()
        return (
            f"Attitude - Roll: {np.rad2deg(roll_rad):.2f} deg, "
            f"Pitch: {np.rad2deg(pitch_rad):.2f} deg, "
            f"Sideslip: {np.rad2deg(truth_data.sideslip_rad):.2f} deg\n"
            f"Body Rates - p: {np.rad2deg(truth_data.p_radps):.2f} deg/s, "
            f"q: {np.rad2deg(truth_data.q_radps):.2f} deg/s, "
            f"r: {np.rad2deg(truth_data.r_radps):.2f} deg/s\n"
            f"Body rate changes - "
            f"dp/dt: {np.rad2deg(d_body_rates_radps2[0]):.2f} deg/s², "
            f"dq/dt: {np.rad2deg(d_body_rates_radps2[1]):.2f} deg/s², "
            f"dr/dt: {np.rad2deg(d_body_rates_radps2[2]):.2f} deg/s²"
        )

    def _log_trim_success(
        self,
        truth_data: SimTruthState,
        d_body_rates_radps2: np.ndarray,
        step: int,
    ) -> None:
        click.secho(f"Trim achieved in {step} steps.", fg="green")
        click.secho(
            self._format_trim_debug_info(truth_data, d_body_rates_radps2), fg="green"
        )

    def _log_trim_failure(
        self,
        truth_data: SimTruthState,
        d_body_rates_radps2: np.ndarray,
    ) -> None:
        click.secho(
            "Failed to achieve trim within the maximum number of steps. "
            + self._format_trim_debug_info(truth_data, d_body_rates_radps2),
            fg="red",
        )


if __name__ == "__main__":
    from jsbsim_sandbox.vehicle_state_visualizer import animate_sim

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

    trim_controller.run_until_trim(
        target_roll_rad=np.deg2rad(60),
        target_pitch_rad=np.deg2rad(0),
        target_sideslip_rad=np.deg2rad(0),
    )

    animate_sim(
        trim_controller.sim_truth_state_history, interval_ms=0.005, frame_step=10
    )
