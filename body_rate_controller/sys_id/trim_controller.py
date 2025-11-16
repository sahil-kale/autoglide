import numpy as np
import click
from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleInitialCond,
    JSBSimVehicleConfig,
    JSBSimSimParams,
)
from vehicle_interface.vehicle_interface import *
from utils.pid import PIDController, PIDConfig


class GliderAttitudeTrimController:
    def __init__(
        self,
        sim: JSBSim_Sandbox,
        dt: float,
        roll_gains: PIDConfig,
        pitch_gains: PIDConfig,
        yaw_gains: PIDConfig,
    ):
        self.sim = sim
        self.dt = dt
        self.roll_controller = PIDController(roll_gains, dt)
        self.pitch_controller = PIDController(pitch_gains, dt)
        self.sideslip_controller = PIDController(yaw_gains, dt)
        self.sim_truth_state_history = []

        self.trim_angle_threshold_rad = np.deg2rad(7.5)  # max attitude error for trim
        self.max_d_body_rate_radps = np.deg2rad(0.04)  # max body rate change per step
        self.in_trim_persistence_counter = 0
        self.in_trim_persistence_threshold_s = (
            1 * self.dt
        )  # number of steps to confirm trim

    def step_trim_to_attitude(
        self,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float,
        truth_data: SimTruthState,
    ) -> ControlCommands:
        euler_angles = truth_data.attitude.get_euler()
        current_roll_rad = euler_angles[0]
        current_pitch_rad = euler_angles[1]
        current_sideslip_rad = truth_data.sideslip_rad

        aileron_cmd = self.roll_controller.step(target_roll_rad, current_roll_rad)
        elevator_cmd = -self.pitch_controller.step(target_pitch_rad, current_pitch_rad)
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
        previous_body_rates = None

        for step in range(max_steps):
            truth_data = self.sim.step(control_commands)
            control_commands = self.step_trim_to_attitude(
                target_roll_rad=target_roll_rad,
                target_pitch_rad=target_pitch_rad,
                target_sideslip_rad=target_sideslip_rad,
                truth_data=truth_data,
            )
            body_rates = np.array(
                [truth_data.p_radps, truth_data.q_radps, truth_data.r_radps]
            )

            if previous_body_rates is None:
                previous_body_rates = body_rates

            d_body_rate_rad_per_s_squared = (
                np.abs(body_rates - previous_body_rates) / self.dt
            )

            d_body_rate_less_than_max = np.all(
                d_body_rate_rad_per_s_squared < self.max_d_body_rate_radps
            )
            attitude_error = np.array(
                [
                    np.abs(target_roll_rad - truth_data.attitude.get_euler()[0]),
                    np.abs(target_pitch_rad - truth_data.attitude.get_euler()[1]),
                    np.abs(target_sideslip_rad - truth_data.sideslip_rad),
                ]
            )

            attitude_error_less_than_threshold = np.all(
                attitude_error < self.trim_angle_threshold_rad
            )

            if d_body_rate_less_than_max and attitude_error_less_than_threshold:
                self.in_trim_persistence_counter += 1 * self.dt
            else:
                self.in_trim_persistence_counter = 0

            if self.in_trim_persistence_counter >= self.in_trim_persistence_threshold_s:
                click.secho(f"Trim achieved in {step} steps.", fg="green")
                click.secho(
                    f"Final Attitude - Roll: {np.rad2deg(truth_data.attitude.get_euler()[0]):.2f} deg, "
                    f"Pitch: {np.rad2deg(truth_data.attitude.get_euler()[1]):.2f} deg, "
                    f"Sideslip: {np.rad2deg(truth_data.sideslip_rad):.2f} deg",
                    fg="green",
                )
                click.secho(
                    f"Final Body Rates - p: {np.rad2deg(truth_data.p_radps):.2f} deg/s, "
                    f"q: {np.rad2deg(truth_data.q_radps):.2f} deg/s, "
                    f"r: {np.rad2deg(truth_data.r_radps):.2f} deg/s",
                    fg="green",
                )
                click.secho(
                    f"Final body rate changes - dp/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[0]):.2f} deg/s², "
                    f"dq/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[1]):.2f} deg/s², "
                    f"dr/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[2]):.2f} deg/s²",
                    fg="green",
                )
                return True

            previous_body_rates = body_rates

        failure_msg = "Failed to achieve trim within the maximum number of steps."
        failure_msg += (
            f" Last body rates: p: {np.rad2deg(truth_data.p_radps):.2f} deg/s, "
        )
        failure_msg += f"q: {np.rad2deg(truth_data.q_radps):.2f} deg/s, "
        failure_msg += f"r: {np.rad2deg(truth_data.r_radps):.2f} deg/s. "
        failure_msg += f"Last body rate changes: dp/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[0]):.2f} deg/s², "
        failure_msg += (
            f"dq/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[1]):.2f} deg/s², "
        )
        failure_msg += (
            f"dr/dt: {np.rad2deg(d_body_rate_rad_per_s_squared[2]):.2f} deg/s²"
        )

        click.secho(failure_msg, fg="red")

        if raise_on_fail:
            raise RuntimeError("Failed to achieve trim.")

        return False


if __name__ == "__main__":
    # Example usage
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
    )

    trim_controller.run_until_trim(
        target_roll_rad=np.deg2rad(60),
        target_pitch_rad=np.deg2rad(0),
        target_sideslip_rad=np.deg2rad(0),
    )

    from jsbsim_sandbox.vehicle_state_visualizer import animate_sim

    # breakpoint()
    animate_sim(
        trim_controller.sim_truth_state_history, interval_ms=0.005, frame_step=10
    )
