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
        sim_truth_state_history: list of SimTruthState from JSBSim_Sandbox, each with:
            - time_s
            - p_radps, q_radps, r_radps
            - aoa_rad (angle of attack)
            - sideslip_rad (beta)
            - control_commands.aileron_deflection_norm
            - control_commands.elevator_deflection_norm
            - control_commands.rudder_deflection_norm
        """
        self.sim_truth_state_history = sim_truth_state_history

        # Populated by identify_model()
        self.A = None          # (5, 5)
        self.B = None          # (5, 3)
        self.c = None          # (5,)
        self.Theta = None      # (9, 5) stacked [A^T; B^T; c^T]
        self.rank = None
        self.singular_values = None
        self.time_vector = None
        self._state_at_t0 = None
        self.dt_id = None      # identification sample time

    def identify_model(self):
        """
        Identify a continuous-time linear model of the form:

            dx_k = A * x_k + B * delta_k + c

        where:
            x = [p, q, r, alpha, beta]^T (deviations from initial state)
            delta = [aileron, elevator, rudder]^T (deflections, deviations from initial)

        We compute state derivatives via finite difference:
            dx_k = (x_{k+1} - x_k) / dt_k

        and then use linear least squares on (x_k, delta_k) to fit A, B, c.
        """
        click.secho("Starting model identification (accel-based, x=[p,q,r,alpha,beta])...", fg="blue")
        state_at_t0 = self.sim_truth_state_history[0]

        time_vector = []
        p_radps_vector = []
        q_radps_vector = []
        r_radps_vector = []
        alpha_rad_vector = []
        beta_rad_vector = []
        aileron_deflection_vector = []
        elevator_deflection_vector = []
        rudder_deflection_vector = []

        # Build time series relative to initial state
        for state in self.sim_truth_state_history:
            time_vector.append(state.time_s - state_at_t0.time_s)

            p_radps_vector.append(state.p_radps - state_at_t0.p_radps)
            q_radps_vector.append(state.q_radps - state_at_t0.q_radps)
            r_radps_vector.append(state.r_radps - state_at_t0.r_radps)

            alpha_rad_vector.append(state.aoa_rad - state_at_t0.aoa_rad)
            beta_rad_vector.append(state.sideslip_rad - state_at_t0.sideslip_rad)

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
        alpha_rad_vector = np.array(alpha_rad_vector)
        beta_rad_vector = np.array(beta_rad_vector)
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

        Let x_k = [p_k, q_k, r_k, alpha_k, beta_k] (5x1 state at time k).
        Let delta_k = [aileron_k, elevator_k, rudder_k] (3x1 control deflections at time k).

        Define:
            dx_k = (x_{k+1} - x_k) / dt_k

        We want:
            dx_k ≈ A * x_k + B * delta_k + c

        Stack the data as:
            Phi_k = [x_k^T, delta_k^T, 1]  -> shape (1, 9)
            D_k   = dx_k^T                 -> shape (1, 5)

        Then solve:
            D ≈ Phi * Theta
            Theta ∈ R^{9x5} = [A^T; B^T; c^T]
        """
        # State matrix x: (N, 5)
        x = np.vstack(
            (
                p_radps_vector,
                q_radps_vector,
                r_radps_vector,
                alpha_rad_vector,
                beta_rad_vector,
            )
        ).T

        # Input matrix delta: (N, 3)
        delta = np.vstack(
            (
                aileron_deflection_vector,
                elevator_deflection_vector,
                rudder_deflection_vector,
            )
        ).T

        # Use k = 0..N-2 for regression, with k+1 as next sample
        x_k = x[:-1, :]          # (N-1, 5)
        x_k1 = x[1:, :]          # (N-1, 5)
        delta_k = delta[:-1, :]  # (N-1, 3)
        time_k = time_vector[:-1]
        time_k1 = time_vector[1:]

        expected_x_shape = (len(time_vector) - 1, 5)
        expected_delta_shape = (len(time_vector) - 1, 3)
        self._verify_array_shape(x_k, expected_x_shape, "x_k")
        self._verify_array_shape(x_k1, expected_x_shape, "x_k+1")
        self._verify_array_shape(delta_k, expected_delta_shape, "delta_k")

        # Compute per-sample dt if needed, but use dt_id for reconstruction
        dt_per_sample = time_k1 - time_k
        if np.any(dt_per_sample <= 0):
            msg = "Non-positive dt encountered in time vector."
            click.secho(msg, fg="red", bold=True)
            raise RuntimeError(msg)

        # Compute dx using per-sample dt
        dx = (x_k1 - x_k) / dt_per_sample[:, None]  # (N-1, 5)

        # Regression matrix Phi: [x_k, delta_k, 1]
        ones_col = np.ones((x_k.shape[0], 1))
        Phi = np.hstack((x_k, delta_k, ones_col))  # (N-1, 9)
        D = dx                                     # (N-1, 5)

        if Phi.shape[0] != D.shape[0]:
            msg = (
                f"Phi and D row count mismatch. "
                f"Phi rows: {Phi.shape[0]}, D rows: {D.shape[0]}"
            )
            click.secho(msg, fg="red", bold=True)
            raise RuntimeError(msg)

        # Solve least squares: Phi * Theta ≈ D
        Theta, ls_residuals, rank, s = np.linalg.lstsq(Phi, D, rcond=None)
        # Theta: (9, 5) = [A^T; B^T; c^T]
        A = Theta[0:5, :].T   # (5, 5)
        B = Theta[5:8, :].T   # (5, 3)
        c = Theta[8, :]       # (5,)

        # Store identified quantities on the instance for later use
        self.A = A
        self.B = B
        self.c = c
        self.Theta = Theta
        self.rank = rank
        self.singular_values = s
        self.time_vector = time_vector
        self._state_at_t0 = state_at_t0

        click.secho(
            "Identified model parameters (dx = A x + B delta + c) with x=[p,q,r,alpha,beta]:",
            fg="green",
        )
        click.secho(f"A matrix (5x5):\n{A}", fg="green")
        click.secho(f"B matrix (5x3):\n{B}", fg="green")
        click.secho(f"c vector (5,):\n{c}", fg="green")
        click.secho(f"Rank of Phi: {rank}", fg="green")
        click.secho(f"Singular values of Phi:\n{s}", fg="green")
        click.secho(f"Identification dt (median): {dt_id:.6f} s", fg="green")

        # One-step debug: reconstruct x_{k+1} from x_k using model
        D_hat = Phi @ Theta                     # (N-1, 5) = predicted dx
        x_k1_hat = x_k + dt_id * D_hat          # use nominal dt

        residuals = x_k1 - x_k1_hat
        ss_res = np.sum(residuals**2, axis=0)
        ss_tot = np.sum((x_k1 - np.mean(x_k1, axis=0)) ** 2, axis=0)
        with np.errstate(divide="ignore", invalid="ignore"):
            r_squared = 1 - (ss_res / ss_tot)

        labels = ["p", "q", "r", "alpha", "beta"]
        click.secho("One-step R^2 per state (debug, reconstructed x_{k+1}):", fg="yellow")
        click.secho(
            ", ".join(f"{name}: {r_squared[i]:.4f}" for i, name in enumerate(labels)),
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
            x_hat[0] = x_true[0]
            dx_hat_k = A * x_hat[k] + B * delta_k + c
            x_hat[k+1] = x_hat[k] + dt_id * dx_hat_k

        where x = [p, q, r, alpha, beta]^T (relative to initial state).
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
        alpha_vec = []
        beta_vec = []
        aileron_vec = []
        elevator_vec = []
        rudder_vec = []

        for s in sim_truth_state_history:
            time_vector.append(s.time_s - state_at_t0.time_s)

            p_vec.append(s.p_radps - state_at_t0.p_radps)
            q_vec.append(s.q_radps - state_at_t0.q_radps)
            r_vec.append(s.r_radps - state_at_t0.r_radps)

            alpha_vec.append(s.aoa_rad - state_at_t0.aoa_rad)
            beta_vec.append(s.sideslip_rad - state_at_t0.sideslip_rad)

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
        x_true = np.vstack(
            (
                np.array(p_vec),
                np.array(q_vec),
                np.array(r_vec),
                np.array(alpha_vec),
                np.array(beta_vec),
            )
        ).T
        delta = np.vstack(
            (np.array(aileron_vec), np.array(elevator_vec), np.array(rudder_vec))
        ).T

        N = x_true.shape[0]
        if N < 2:
            raise RuntimeError("Not enough samples for rollout.")

        # Multi-step prediction using model only
        x_hat = np.zeros_like(x_true)
        x_hat[0, :] = x_true[0, :]  # start from true initial state

        dt = self.dt_id
        for k in range(N - 1):
            dx_hat_k = self.A @ x_hat[k, :] + self.B @ delta[k, :] + self.c
            x_hat[k + 1, :] = x_hat[k, :] + dt * dx_hat_k

        return time_vector, x_true, x_hat, state_at_t0

    def plot_rollout_vs_actual(
        self,
        sim_truth_state_history: list | None = None,
        save_path: str | None = None,
        show: bool = True,
    ):
        """
        Plot actual vs multi-step model rollout for:
            p, q, r, alpha, beta

        Uses the acceleration-based model and the dt identified in identify_model().
        """
        (
            time_vector,
            x_true_rel,
            x_hat_rel,
            state_at_t0,
        ) = self.rollout_model(sim_truth_state_history)

        # Recover absolute values by adding trim offsets
        p0 = state_at_t0.p_radps
        q0 = state_at_t0.q_radps
        r0 = state_at_t0.r_radps
        alpha0 = state_at_t0.aoa_rad
        beta0 = state_at_t0.sideslip_rad

        offsets = np.array([p0, q0, r0, alpha0, beta0])
        x_true_abs = x_true_rel + offsets
        x_hat_abs = x_hat_rel + offsets

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

        labels_full = ["p (rad/s)", "q (rad/s)", "r (rad/s)", "alpha (rad)", "beta (rad)"]

        # Compute metrics per channel over the *full* trajectory
        nrmse_vals = [nrmse(x_true_abs[:, i], x_hat_abs[:, i]) for i in range(5)]
        r2_vals = [r2_score(x_true_abs[:, i], x_hat_abs[:, i]) for i in range(5)]

        click.secho("Multi-step rollout metrics (full trajectory, x=[p,q,r,alpha,beta]):", fg="cyan")
        for i, name in enumerate(["p", "q", "r", "alpha", "beta"]):
            click.secho(
                f"{name:>6}  NRMSE: {nrmse_vals[i]:.4f},  R^2: {r2_vals[i]:.4f}",
                fg="cyan",
            )

        # Plot 5x1
        fig, axs = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
        for i, ax in enumerate(axs):
            ax.plot(
                time_vector,
                x_true_abs[:, i],
                label="Actual",
                linewidth=1,
            )
            ax.plot(
                time_vector,
                x_hat_abs[:, i],
                label="Model rollout",
                linewidth=1,
            )
            ax.set_ylabel(labels_full[i])
            ax.grid(True)
            ax.legend()
            ax.set_title(
                f"{labels_full[i]} — NRMSE = {nrmse_vals[i]:.3f}, $R^2$ = {r2_vals[i]:.3f}"
            )

        axs[-1].set_xlabel("Time (s)")
        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=200)

        if show:
            plt.show()

        return {
            "time": time_vector,
            "x_true_abs": x_true_abs,
            "x_hat_abs": x_hat_abs,
            "nrmse": {
                "p": nrmse_vals[0],
                "q": nrmse_vals[1],
                "r": nrmse_vals[2],
                "alpha": nrmse_vals[3],
                "beta": nrmse_vals[4],
            },
            "r2": {
                "p": r2_vals[0],
                "q": r2_vals[1],
                "r": r2_vals[2],
                "alpha": r2_vals[3],
                "beta": r2_vals[4],
            },
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
        roll_rad=np.deg2rad(0),
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
    model_identifier.plot_rollout_vs_actual()
