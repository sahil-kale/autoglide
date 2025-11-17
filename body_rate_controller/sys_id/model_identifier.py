from jsbsim_sandbox.sandbox_sim import (
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
    JSBSim_Sandbox,
)

from utils.pid import PIDConfig
from jsbsim_sandbox.vehicle_state_visualizer import animate_sim
import numpy as np

from body_rate_controller.sys_id.trim_controller import (
    GliderAttitudeTrimController,
    TrimConfig,
    TrimTarget,
)

from body_rate_controller.sys_id.body_rate_model_perturber import (
    BodyRateModelPerturber,
)
import click


class BodyRateModelIdentifier:
    def __init__(self, sim_truth_state_history: list):
        self.sim_truth_state_history = sim_truth_state_history

    def identify_model(self):
        click.secho("Starting model identification...", fg="blue")
        state_at_t0 = self.sim_truth_state_history[0]

        time_vector = []
        p_radps_vector = []
        q_radps_vector = []
        r_radps_vector = []
        aileron_deflection_vector = []
        elevator_deflection_vector = []
        rudder_deflection_vector = []

        for state in self.sim_truth_state_history:
            time_vector.append(state.time_s - state_at_t0.time_s)
            p_radps_vector.append(state.p_radps - state_at_t0.p_radps)
            q_radps_vector.append(state.q_radps - state_at_t0.q_radps)
            r_radps_vector.append(state.r_radps - state_at_t0.r_radps)
            aileron_deflection_vector.append(
                state.control_commands.aileron_deflection_norm
                - state_at_t0.control_commands.aileron_deflection_norm
            )
            elevator_deflection_vector.append(
                state.control_commands.elevator_deflection_norm
                - state_at_t0.control_commands.elevator_deflection_norm
            )
            rudder_deflection_vector.append(
                state.control_commands.rudder_deflection_norm
                - state_at_t0.control_commands.rudder_deflection_norm
            )

        # Convert lists to numpy arrays for easier manipulation
        time_vector = np.array(time_vector)
        p_radps_vector = np.array(p_radps_vector)
        q_radps_vector = np.array(q_radps_vector)
        r_radps_vector = np.array(r_radps_vector)
        aileron_deflection_vector = np.array(aileron_deflection_vector)
        elevator_deflection_vector = np.array(elevator_deflection_vector)
        rudder_deflection_vector = np.array(rudder_deflection_vector)

        """
        Structure of model to identify:
        Let omega = [p, q, r] (body rates).
        Let delta = [aileron, elevator, rudder] (control deflections).
        Let k be the time step index, and N be the total number of time steps minus 2.

        We want to find:
        omega_{k+1} = A * omega_k + B * delta_k

        Using least squares, we can stack the data to solve for A and B.
        Let Phi = [omega, delta] (becomes a N x 6 matrix).
        Let Y = [omega_next] (becomes a N x 3 matrix), where omega_next is omega shifted by one time step.
        Let Theta = [A; B] (Where A, B are both 3x3 matrices, stacked vertically to form a 6x3 matrix).
        We want to solve Y_est = Phi * Theta, formally stated as:
        argmin || Y - Phi * Theta ||^2 over Theta
        """
        omega = np.vstack((p_radps_vector, q_radps_vector, r_radps_vector)).T
        omega_next = omega[1:, :]
        # remove the last element inside omega
        omega = omega[:-1, :]
        expected_omega_shape = (len(time_vector) - 1, 3)
        self._verify_array_shape(omega, expected_omega_shape, "Omega")

        expected_omega_next_shape = (len(time_vector) - 1, 3)
        self._verify_array_shape(omega_next, expected_omega_next_shape, "Omega Next")

        delta = np.vstack(
            (
                aileron_deflection_vector,
                elevator_deflection_vector,
                rudder_deflection_vector,
            )
        ).T
        delta = delta[:-1, :]
        expected_delta_shape = (len(time_vector) - 1, 3)
        self._verify_array_shape(delta, expected_delta_shape, "Delta")

        Phi = np.hstack((omega, delta))  # N x 6
        Y = omega_next  # N x 3

        if Phi.shape[0] != Y.shape[0]:
            raise RuntimeError(
                click.secho(
                    f"Phi and Y row count mismatch. Phi rows: {Phi.shape[0]}, Y rows: {Y.shape[0]}",
                    fg="red",
                    bold=True,
                )
            )

        # Solve for Theta using least squares
        Theta, residuals, rank, s = np.linalg.lstsq(Phi, Y, rcond=None)
        A = Theta[:3, :].T  # 3x3
        B = Theta[3:, :].T  # 3x3
        click.secho("Identified model parameters:", fg="green")
        click.secho(f"A matrix:\n{A}", fg="green")
        click.secho(f"B matrix:\n{B}", fg="green")

        # Fit stats
        Y_est = Phi @ Theta  # N x 3
        residuals = Y - Y_est
        ss_res = np.sum(residuals**2, axis=0)
        ss_tot = np.sum((Y - np.mean(Y, axis=0)) ** 2, axis=0)
        r_squared = 1 - (ss_res / ss_tot)
        click.secho(f"R-squared values for each body rate:", fg="green")
        click.secho(
            f"p: {r_squared[0]:.4f}, q: {r_squared[1]:.4f}, r: {r_squared[2]:.4f}",
            fg="green",
        )

    @staticmethod
    def _verify_array_shape(array, expected_shape, array_name):
        if array.shape != expected_shape:
            raise RuntimeError(
                click.secho(
                    f"{array_name} shape mismatch. Expected {expected_shape}, got {array.shape}",
                    fg="red",
                    bold=True,
                )
            )


if __name__ == "__main__":
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
        roll_rad=np.deg2rad(30),
        pitch_rad=np.deg2rad(-10),
        sideslip_rad=np.deg2rad(0),
    )

    trim_controller.run_until_trim(
        trim_target=trim_target,
    )

    time_at_perturb_start_s = sim.get_sim_time_s()

    perturber = BodyRateModelPerturber(sim, trim_controller, trim_target)
    perturber.run()

    history = sim.sim_truth_state_history
    idx_of_perturb_start = time_at_perturb_start_s // sim_params.dt_s
    history = history[int(idx_of_perturb_start) :]

    model_identifier = BodyRateModelIdentifier(history)
    model_identifier.identify_model()
