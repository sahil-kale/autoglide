from jsbsim_sandbox.sandbox_sim import (
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
    JSBSim_Sandbox,
)

from utils.pid import PIDConfig
from body_rate_controller.sys_id.trim_controller import (
    GliderAttitudeTrimController,
    TrimConfig,
    TrimTarget,
)

from body_rate_controller.sys_id.body_rate_model_perturber import (
    BodyRateModelPerturber,
)

import numpy as np
import matplotlib.pyplot as plt
import click


class BodyRateModelIdentifier:
    def __init__(self, sim_truth_state_history: list):
        """
        sim_truth_state_history: list of states from JSBSim_Sandbox, each with:
            - time_s
            - p_radps, q_radps, r_radps
            - control_commands.aileron_deflection_norm
            - control_commands.elevator_deflection_norm
            - control_commands.rudder_deflection_norm
        """
        self.sim_truth_state_history = sim_truth_state_history

        # Populated by identify_model()
        self.A = None          # (3, 3)
        self.B = None          # (3, 3)
        self.c = None          # (3,)
        self.Theta = None      # (7, 3) stacked [A^T; B^T; c^T]
        self.rank = None
        self.singular_values = None
        self.time_vector = None
        self._state_at_t0 = None
        self.dt_id = None      # identification sample time

    def identify_model(self):
        """
        Identify a discrete-time model of the form (in continuous-time-ish form):

            domega_k = A * omega_k + B * delta_k + c

        where:
            omega = [p, q, r]^T (body rates, deviations from initial state)
            delta = [aileron, elevator, rudder]^T (deflections, deviations from initial)

        We compute angular accelerations via finite difference:
            domega_k = (omega_{k+1} - omega_k) / dt

        and then use linear least squares on (omega_k, delta_k) to fit A, B, c.
        """
        click.secho("Starting model identification (acceleration-based)...", fg="blue")
        state_at_t0 = self.sim_truth_state_history[0]

        time_vector = []
        p_radps_vector = []
        q_radps_vector = []
        r_radps_vector = []
        aileron_deflection_vector = []
        elevator_deflection_vector = []
        rudder_deflection_vector = []

        # Build time series relative to initial state
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

        # Convert lists to numpy arrays
        time_vector = np.array(time_vector)
        p_radps_vector = np.array(p_radps_vector)
        q_radps_vector = np.array(q_radps_vector)
        r_radps_vector = np.array(r_radps_vector)
        aileron_deflection_vector = np.array(aileron_deflection_vector)
        elevator_deflection_vector = np.array(elevator_deflection_vector)
        rudder_deflection_vector = np.array(rudder_deflection_vector)

        # Estimate dt for identification (assume roughly constant)
        if len(time_vector) < 2:
            raise RuntimeError("Not enough samples to estimate dt.")
        dt_candidates = np.diff(time_vector)
        dt_id = float(np.median(dt_candidates))
        self.dt_id = dt_id

        """
        Structure of model to identify:

        Let omega_k = [p_k, q_k, r_k] (body rates at time k).
        Let delta_k = [aileron_k, elevator_k, rudder_k] (control deflections at time k).

        Define:
            domega_k = (omega_{k+1} - omega_k) / dt

        We want:
            domega_k ≈ A * omega_k + B * delta_k + c

        Stack the data as:
            Phi_k = [omega_k^T, delta_k^T, 1]  -> shape (1, 7)
            D_k   = domega_k^T                 -> shape (1, 3)

        Then solve:
            D ≈ Phi * Theta
            Theta ∈ R^{7x3} = [A^T; B^T; c^T]
        """
        omega = np.vstack((p_radps_vector, q_radps_vector, r_radps_vector)).T  # (N, 3)
        delta = np.vstack(
            (
                aileron_deflection_vector,
                elevator_deflection_vector,
                rudder_deflection_vector,
            )
        ).T  # (N, 3)

        # Use k = 0..N-2 for regression, with k+1 as next sample
        omega_k = omega[:-1, :]          # (N-1, 3)
        omega_k1 = omega[1:, :]          # (N-1, 3)
        delta_k = delta[:-1, :]          # (N-1, 3)
        time_k = time_vector[:-1]
        time_k1 = time_vector[1:]

        expected_shape = (len(time_vector) - 1, 3)
        self._verify_array_shape(omega_k, expected_shape, "Omega_k")
        self._verify_array_shape(omega_k1, expected_shape, "Omega_k+1")
        self._verify_array_shape(delta_k, expected_shape, "Delta_k")

        # Compute per-sample dt if needed, but use dt_id for regression
        dt_per_sample = time_k1 - time_k
        # Avoid division by zero
        if np.any(dt_per_sample <= 0):
            msg = "Non-positive dt encountered in time vector."
            click.secho(msg, fg="red", bold=True)
            raise RuntimeError(msg)

        # Compute domega using per-sample dt but treat dt_id as nominal
        domega = (omega_k1 - omega_k) / dt_per_sample[:, None]  # (N-1, 3)

        # Regression matrix Phi: [omega_k, delta_k, 1]
        ones_col = np.ones((omega_k.shape[0], 1))
        Phi = np.hstack((omega_k, delta_k, ones_col))  # (N-1, 7)
        D = domega                                     # (N-1, 3)

        if Phi.shape[0] != D.shape[0]:
            msg = (
                f"Phi and D row count mismatch. "
                f"Phi rows: {Phi.shape[0]}, D rows: {D.shape[0]}"
            )
            click.secho(msg, fg="red", bold=True)
            raise RuntimeError(msg)

        # Solve least squares: Phi * Theta ≈ D
        Theta, ls_residuals, rank, s = np.linalg.lstsq(Phi, D, rcond=None)
        # Theta: (7, 3) = [A^T; B^T; c^T]
        A = Theta[0:3, :].T   # (3, 3)
        B = Theta[3:6, :].T   # (3, 3)
        c = Theta[6, :]       # (3,)

        # Store identified quantities on the instance for later use
        self.A = A
        self.B = B
        self.c = c
        self.Theta = Theta
        self.rank = rank
        self.singular_values = s
        self.time_vector = time_vector
        self._state_at_t0 = state_at_t0

        click.secho("Identified model parameters (domega = A*omega + B*delta + c):", fg="green")
        click.secho(f"A matrix:\n{A}", fg="green")
        click.secho(f"B matrix:\n{B}", fg="green")
        click.secho(f"c vector:\n{c}", fg="green")
        click.secho(f"Rank of Phi: {rank}", fg="green")
        click.secho(f"Singular values of Phi:\n{s}", fg="green")
        click.secho(f"Identification dt (median): {dt_id:.6f} s", fg="green")

        # One-step debug: reconstruct omega_{k+1} from omega_k using model
        D_hat = Phi @ Theta                     # (N-1, 3) = predicted domega
        omega_k1_hat = omega_k + dt_id * D_hat  # use nominal dt

        residuals = omega_k1 - omega_k1_hat
        ss_res = np.sum(residuals**2, axis=0)
        ss_tot = np.sum((omega_k1 - np.mean(omega_k1, axis=0)) ** 2, axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            r_squared = 1 - (ss_res / ss_tot)

        click.secho("One-step R^2 (debug, based on reconstructed omega_{k+1}):", fg="yellow")
        click.secho(
            f"p: {r_squared[0]:.4f}, q: {r_squared[1]:.4f}, r: {r_squared[2]:.4f}",
            fg="yellow",
        )

        return {
            "A": A,
            "B": B,
            "c": c,
            "Theta": Theta,
            "rank": rank,
            "singular_values": s,
            "one_step_r2": r_squared,
            "dt_id": dt_id,
        }

    def rollout_model(self, sim_truth_state_history: list | None = None):
        """
        Multi-step rollout using the identified *acceleration-based* model.

        We use:
            omega_hat[0] = omega_true[0]
            domega_hat_k = A * omega_hat[k] + B * delta_k + c
            omega_hat[k+1] = omega_hat[k] + dt_id * domega_hat_k

        This tests whether the model captures the true dynamics over time.
        """
        if sim_truth_state_history is None:
            sim_truth_state_history = self.sim_truth_state_history

        if self.A is None or self.B is None or self.c is None:
            raise RuntimeError("Model not identified yet. Call identify_model() first.")

        if self.dt_id is None:
            raise RuntimeError("Identification dt is unknown. Run identify_model() first.")

        state_at_t0 = sim_truth_state_history[0]

        time_vector = []
        p_vec = []
        q_vec = []
        r_vec = []
        aileron_vec = []
        elevator_vec = []
        rudder_vec = []

        for s in sim_truth_state_history:
            time_vector.append(s.time_s - state_at_t0.time_s)
            p_vec.append(s.p_radps - state_at_t0.p_radps)
            q_vec.append(s.q_radps - state_at_t0.q_radps)
            r_vec.append(s.r_radps - state_at_t0.r_radps)
            aileron_vec.append(
                s.control_commands.aileron_deflection_norm
                - state_at_t0.control_commands.aileron_deflection_norm
            )
            elevator_vec.append(
                s.control_commands.elevator_deflection_norm
                - state_at_t0.control_commands.elevator_deflection_norm
            )
            rudder_vec.append(
                s.control_commands.rudder_deflection_norm
                - state_at_t0.control_commands.rudder_deflection_norm
            )

        time_vector = np.array(time_vector)
        omega_true = np.vstack((np.array(p_vec), np.array(q_vec), np.array(r_vec))).T
        delta = np.vstack(
            (np.array(aileron_vec), np.array(elevator_vec), np.array(rudder_vec))
        ).T

        N = omega_true.shape[0]
        if N < 2:
            raise RuntimeError("Not enough samples for rollout.")

        # Multi-step prediction using model only
        omega_hat = np.zeros_like(omega_true)
        omega_hat[0, :] = omega_true[0, :]  # start from true initial state

        dt = self.dt_id
        for k in range(N - 1):
            domega_hat_k = self.A @ omega_hat[k, :] + self.B @ delta[k, :] + self.c
            omega_hat[k + 1, :] = omega_hat[k, :] + dt * domega_hat_k

        return time_vector, omega_true, omega_hat, state_at_t0

    def plot_rollout_vs_actual_body_rates(
        self,
        sim_truth_state_history: list | None = None,
        save_path: str | None = None,
        show: bool = True,
    ):
        """
        Plot actual vs multi-step model rollout for body rates (p, q, r).

        Uses the acceleration-based model and the dt identified in identify_model().
        """
        (
            time_vector,
            omega_true_rel,
            omega_hat_rel,
            state_at_t0,
        ) = self.rollout_model(sim_truth_state_history)

        # Convert relative rates back to absolute for plotting by adding initial rate
        p0 = state_at_t0.p_radps
        q0 = state_at_t0.q_radps
        r0 = state_at_t0.r_radps

        omega_true_abs = omega_true_rel + np.array([p0, q0, r0])
        omega_hat_abs = omega_hat_rel + np.array([p0, q0, r0])

        def nrmse(y_true, y_pred):
            num = np.linalg.norm(y_true - y_pred)
            denom = np.linalg.norm(y_true - np.mean(y_true))
            if denom == 0:
                return float("nan")
            return num / denom

        def r2_score(y_true, y_pred):
            ss_res = np.sum((y_true - y_pred) ** 2)
            ss_tot = np.sum((y_true - np.mean(y_true)) ** 2)
            if ss_tot == 0:
                return float("nan")
            return 1.0 - ss_res / ss_tot

        # Compute metrics per channel over the *full* trajectory
        nrmse_p = nrmse(omega_true_abs[:, 0], omega_hat_abs[:, 0])
        nrmse_q = nrmse(omega_true_abs[:, 1], omega_hat_abs[:, 1])
        nrmse_r = nrmse(omega_true_abs[:, 2], omega_hat_abs[:, 2])

        r2_p = r2_score(omega_true_abs[:, 0], omega_hat_abs[:, 0])
        r2_q = r2_score(omega_true_abs[:, 1], omega_hat_abs[:, 1])
        r2_r = r2_score(omega_true_abs[:, 2], omega_hat_abs[:, 2])

        click.secho("Multi-step rollout metrics (full trajectory, accel-based model):", fg="cyan")
        click.secho(
            f"NRMSE   p: {nrmse_p:.4f}, q: {nrmse_q:.4f}, r: {nrmse_r:.4f}",
            fg="cyan",
        )
        click.secho(
            f"R^2     p: {r2_p:.4f}, q: {r2_q:.4f}, r: {r2_r:.4f}",
            fg="cyan",
        )

        # Plot 3x1
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
        labels = ["p (rad/s)", "q (rad/s)", "r (rad/s)"]
        nrmse_vals = [nrmse_p, nrmse_q, nrmse_r]
        r2_vals = [r2_p, r2_q, r2_r]

        for i, ax in enumerate(axs):
            ax.plot(
                time_vector,
                omega_true_abs[:, i],
                label="Actual",
                linewidth=1,
            )
            ax.plot(
                time_vector,
                omega_hat_abs[:, i],
                label="Model rollout",
                linewidth=1,
            )
            ax.set_ylabel(labels[i])
            ax.grid(True)
            ax.legend()
            ax.set_title(
                f"{labels[i]} — NRMSE = {nrmse_vals[i]:.3f}, $R^2$ = {r2_vals[i]:.3f}"
            )

        axs[-1].set_xlabel("Time (s)")
        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=200)

        if show:
            plt.show()

        return {
            "time": time_vector,
            "omega_true_abs": omega_true_abs,
            "omega_hat_abs": omega_hat_abs,
            "nrmse": {"p": nrmse_p, "q": nrmse_q, "r": nrmse_r},
            "r2": {"p": r2_p, "q": r2_q, "r": r2_r},
        }

    @staticmethod
    def _verify_array_shape(array, expected_shape, array_name):
        if array.shape != expected_shape:
            msg = (
                f"{array_name} shape mismatch. "
                f"Expected {expected_shape}, got {array.shape}"
            )
            click.secho(msg, fg="red", bold=True)
            raise RuntimeError(msg)


if __name__ == "__main__":
    # --- JSBSim setup ---
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

    # --- Trim to a desired attitude ---
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

    trim_controller.run_until_trim(trim_target=trim_target)

    time_at_perturb_start_s = sim.get_sim_time_s()

    # --- Run perturbation maneuver (PRBS/steps) for sys-ID ---
    perturber = BodyRateModelPerturber(sim, trim_controller, trim_target)
    perturber.run()

    # --- Extract history from perturb start onward ---
    history = sim.sim_truth_state_history
    idx_of_perturb_start = int(time_at_perturb_start_s // sim_params.dt_s)
    history = history[idx_of_perturb_start:]

    # --- Identify model and validate with rollout ---
    model_identifier = BodyRateModelIdentifier(history)
    model_identifier.identify_model()
    model_identifier.plot_rollout_vs_actual_body_rates()
