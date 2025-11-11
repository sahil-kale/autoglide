#!/usr/bin/env python3
"""
perturber.py

1) trim_for_operating_point: find trim control biases for a target roll/pitch (inertial)
   while driving sideslip beta → 0. Ignores yaw requirements (coordinated-turn assumption).

2) run_perturbation: apply PRBS perturbations on aileron, elevator, rudder (sequential segments).

3) Log timeseries to CSV compatible with your existing sys-ID tooling:
   header: t,phi,theta,p,q,r,beta,u_a,u_e,u_r,vt,h
"""

import argparse
import math
import os
import sys
import numpy as np

# --- repo imports (as in your sandbox file) ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import units
from vehicle_interface.vehicle_interface import (
    ControlCommands,
    SimTruthState,
)  # noqa: E402

# If you place this file standalone, replace the above with:
from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleInitialCond,
    JSBSimVehicleConfig,
    JSBSimSimParams,
)


# -------------------- helpers --------------------


def compute_beta_rad(fdm) -> float:
    """Sideslip β ≈ atan2(v, u) in body frame. JSBSim velocities in ft/s."""
    u_fps = fdm["velocities/u-fps"]
    v_fps = fdm["velocities/v-fps"]
    return math.atan2(v_fps, u_fps)


def prbs_sequence(num_steps: int, bit_len: int, amplitude: float, seed: int = 1):
    """Simple ±amplitude PRBS-like sequence with piecewise-constant segments."""
    rng = np.random.default_rng(seed)
    num_bits = max(1, int(np.ceil(num_steps / bit_len)))
    bits = rng.choice([-1.0, 1.0], size=num_bits)
    u = np.repeat(bits, bit_len)[:num_steps]
    return amplitude * u


# -------------------- 1) TRIM --------------------


def trim_for_operating_point(
    sim: JSBSim_Sandbox,
    phi_ref_rad: float,
    theta_ref_rad: float,
    settle_s: float = 1.0,
    hold_s: float = 5.0,
    dt: float = 0.01,
    gains=None,
):
    """
    Drive (phi, theta) → (phi_ref, theta_ref) and beta → 0 using a gentle PD/PI trim-hold.
    We ignore yaw constraints; any r that appears is not controlled here.

    Returns:
      biases: dict with 'u_a_bias', 'u_e_bias', 'u_r_bias' (normalized)
      final_state: dict snapshot at the end (phi, theta, beta, p, q, r)
    """
    if gains is None:
        gains = {
            "kp_phi": 0.8,
            "kd_p": 0.25,
            "kp_theta": 0.8,
            "kd_q": 0.25,
            "kp_beta": 0.4,
            "ki_beta": 0.05,  # small integral to wipe steady β
        }

    total_steps = int(round((settle_s + hold_s) / dt))
    beta_int = 0.0

    cc = ControlCommands(
        aileron_deflection_norm=0.0,
        elevator_deflection_norm=0.0,
        rudder_deflection_norm=0.0,
        spoiler_deflection=0.0,
    )

    # Move to desired inertial attitude quickly then hold + clean up β
    trim_hist = {
        "u_a": np.zeros(total_steps),
        "u_e": np.zeros(total_steps),
        "u_r": np.zeros(total_steps),
    }

    for k in range(total_steps):
        # Measurements
        phi = sim.fdm["attitude/phi-rad"]
        theta = sim.fdm["attitude/theta-rad"]
        p = sim.fdm["velocities/p-rad_sec"]
        q = sim.fdm["velocities/q-rad_sec"]
        r = sim.fdm["velocities/r-rad_sec"]
        beta = compute_beta_rad(sim.fdm)

        # Errors (inertial for φ, θ; body for β)
        e_phi = phi_ref_rad - phi
        e_theta = theta_ref_rad - theta
        e_beta = -beta

        # Simple rate-damped proportional control to bring to ref
        u_a_cmd = gains["kp_phi"] * e_phi - gains["kd_p"] * p
        u_e_cmd = gains["kp_theta"] * e_theta - gains["kd_q"] * q

        # Rudder: keep β ≈ 0 with a little I
        beta_int += e_beta * dt
        u_r_cmd = gains["kp_beta"] * e_beta + gains["ki_beta"] * beta_int

        # Apply
        cc.aileron_deflection_norm = float(np.clip(u_a_cmd, -1.0, 1.0))
        cc.elevator_deflection_norm = float(np.clip(u_e_cmd, -1.0, 1.0))
        cc.rudder_deflection_norm = float(np.clip(u_r_cmd, -1.0, 1.0))
        sim.step(cc)

        trim_hist["u_a"][k] = cc.aileron_deflection_norm
        trim_hist["u_e"][k] = cc.elevator_deflection_norm
        trim_hist["u_r"][k] = cc.rudder_deflection_norm

    # Take average of the last 1 s as the "trim biases"
    avg_len = max(2, int(round(1.0 / dt)))
    u_a_bias = float(np.mean(trim_hist["u_a"][-avg_len:]))
    u_e_bias = float(np.mean(trim_hist["u_e"][-avg_len:]))
    u_r_bias = float(np.mean(trim_hist["u_r"][-avg_len:]))

    # Final snapshot
    final_state = dict(
        phi=float(sim.fdm["attitude/phi-rad"]),
        theta=float(sim.fdm["attitude/theta-rad"]),
        p=float(sim.fdm["velocities/p-rad_sec"]),
        q=float(sim.fdm["velocities/q-rad_sec"]),
        r=float(sim.fdm["velocities/r-rad_sec"]),
        beta=float(compute_beta_rad(sim.fdm)),
    )

    biases = dict(u_a_bias=u_a_bias, u_e_bias=u_e_bias, u_r_bias=u_r_bias)
    return biases, final_state


# -------------------- 2) PERTURBATION --------------------


def build_sequential_prbs(
    dt: float,
    settle_after_trim_s: float,
    seg_s: float,
    prbs_bit_s: float,
    amp_a: float,
    amp_e: float,
    amp_r: float,
    seed: int = 1,
):
    """
    Build 3 sequential PRBS segments separated by an initial settle period.
    Returns time array and control arrays u_a, u_e, u_r (all length N).
    """
    n_settle = int(round(settle_after_trim_s / dt))
    n_seg = int(round(seg_s / dt))
    bit_len = max(1, int(round(prbs_bit_s / dt)))

    # Total length: settle + 3 segments
    n_total = n_settle + 3 * n_seg
    t = np.arange(n_total) * dt

    u_a = np.zeros(n_total)
    u_e = np.zeros(n_total)
    u_r = np.zeros(n_total)

    # Aileron PRBS
    if n_seg > 0:
        u_a[n_settle : n_settle + n_seg] = prbs_sequence(
            n_seg, bit_len, amp_a, seed + 1
        )

    # Elevator PRBS
    idx_e = n_settle + n_seg
    if n_seg > 0:
        u_e[idx_e : idx_e + n_seg] = prbs_sequence(n_seg, bit_len, amp_e, seed + 2)

    # Rudder PRBS
    idx_r = n_settle + 2 * n_seg
    if n_seg > 0:
        u_r[idx_r : idx_r + n_seg] = prbs_sequence(n_seg, bit_len, amp_r, seed + 3)

    return t, u_a, u_e, u_r


# -------------------- 3) RUN + LOG --------------------


def run_perturbation_and_log(
    sim: JSBSim_Sandbox,
    biases: dict,
    t: np.ndarray,
    u_a: np.ndarray,
    u_e: np.ndarray,
    u_r: np.ndarray,
    out_csv: str,
):
    """
    Runs the sim with (bias + perturbation) commands and logs CSV:
    t,phi,theta,p,q,r,beta,u_a,u_e,u_r,vt,h
    """
    os.makedirs(os.path.dirname(out_csv) or ".", exist_ok=True)

    N = len(t)
    dt = float(sim.sim_params.dt_s)

    # Storage
    phi = np.zeros(N)
    theta = np.zeros(N)
    p = np.zeros(N)
    q = np.zeros(N)
    r = np.zeros(N)
    beta = np.zeros(N)
    vt = np.zeros(N)
    h = np.zeros(N)

    # Command container
    cc = ControlCommands(
        aileron_deflection_norm=0.0,
        elevator_deflection_norm=0.0,
        rudder_deflection_norm=0.0,
        spoiler_deflection=0.0,
    )

    for k in range(N):
        # Apply biases + perturbations, clipped to [-1, 1]
        cc.aileron_deflection_norm = float(
            np.clip(biases["u_a_bias"] + u_a[k], -1.0, 1.0)
        )
        cc.elevator_deflection_norm = float(
            np.clip(biases["u_e_bias"] + u_e[k], -1.0, 1.0)
        )
        cc.rudder_deflection_norm = float(
            np.clip(biases["u_r_bias"] + u_r[k], -1.0, 1.0)
        )

        sim.step(cc)

        # Log
        phi[k] = sim.fdm["attitude/phi-rad"]
        theta[k] = sim.fdm["attitude/theta-rad"]
        p[k] = sim.fdm["velocities/p-rad_sec"]
        q[k] = sim.fdm["velocities/q-rad_sec"]
        r[k] = sim.fdm["velocities/r-rad_sec"]
        beta[k] = compute_beta_rad(sim.fdm)
        vt[k] = units.knots_to_mps(sim.fdm["velocities/vtrue-kts"])
        h[k] = units.feet_to_meters(sim.fdm["position/h-sl-ft"])

        # Keep real-time-ish spacing if your dt differs from array spacing
        # (not needed here; sim.dt is fixed)

    # Save CSV
    header = "t,phi,theta,p,q,r,beta,u_a,u_e,u_r,vt,h"
    data = np.column_stack(
        [
            t,
            phi,
            theta,
            p,
            q,
            r,
            beta,
            biases["u_a_bias"] + u_a,
            biases["u_e_bias"] + u_e,
            biases["u_r_bias"] + u_r,
            vt,
            h,
        ]
    )
    np.savetxt(out_csv, data, delimiter=",", header=header, comments="")
    print(f"[perturber] wrote {out_csv}")


# -------------------- CLI --------------------


def main():
    ap = argparse.ArgumentParser(
        description="Trim to (phi,theta) & β≈0, then perturb and log for system ID."
    )
    ap.add_argument(
        "--aircraft", default="ask21", help="JSBSim model name (e.g., 'glider')"
    )
    ap.add_argument(
        "--jsbsim_root", default="jsbsim_sandbox/", help="JSBSim root dir (optional)"
    )
    ap.add_argument("--dt", type=float, default=0.01, help="Simulation dt [s]")

    # Initial condition (position/airspeed/heading)
    ap.add_argument("--h0_m", type=float, default=2000.0)
    ap.add_argument("--vt0_mps", type=float, default=40.0)
    ap.add_argument("--lat0_deg", type=float, default=37.4275)
    ap.add_argument("--lon0_deg", type=float, default=-122.1697)
    ap.add_argument("--psi0_rad", type=float, default=0.0)

    # Target operating point (inertial)
    ap.add_argument("--phi_ref_deg", type=float, default=0.0, help="Target roll [deg]")
    ap.add_argument(
        "--theta_ref_deg", type=float, default=0.0, help="Target pitch [deg]"
    )

    # Trim/hold timing
    ap.add_argument(
        "--trim_settle_s", type=float, default=10.0, help="Pre-hold settle seconds"
    )
    ap.add_argument(
        "--trim_hold_s",
        type=float,
        default=5.0,
        help="Hold seconds for averaging biases",
    )

    # Perturbation design
    ap.add_argument(
        "--post_trim_settle_s",
        type=float,
        default=2.0,
        help="Settle after trim before PRBS",
    )
    ap.add_argument(
        "--seg_s", type=float, default=20.0, help="Per-axis PRBS segment length [s]"
    )
    ap.add_argument("--prbs_bit_s", type=float, default=0.5, help="PRBS bit length [s]")
    ap.add_argument(
        "--amp_a", type=float, default=0.05, help="Aileron amplitude (normalized)"
    )
    ap.add_argument(
        "--amp_e", type=float, default=0.05, help="Elevator amplitude (normalized)"
    )
    ap.add_argument(
        "--amp_r", type=float, default=0.05, help="Rudder amplitude (normalized)"
    )
    ap.add_argument("--seed", type=int, default=1, help="PRBS seed")

    ap.add_argument(
        "--out_csv", default="output/sysid/sysid_log.csv", help="Output CSV path"
    )

    args = ap.parse_args()

    # Build sandbox
    initial_cond = JSBSimVehicleInitialCond(
        h0_m=args.h0_m,
        vt0_mps=args.vt0_mps,
        lat0_deg=args.lat0_deg,
        lon0_deg=args.lon0_deg,
        psi0_rad=args.psi0_rad,
        phi0_rad=0.0,
        theta0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=args.dt)
    vehicle_config = JSBSimVehicleConfig(
        model_name=args.aircraft,
        root_dir=args.jsbsim_root,
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )
    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)

    # 1) Trim
    phi_ref = math.radians(args.phi_ref_deg)
    theta_ref = math.radians(args.theta_ref_deg)
    biases, snap = trim_for_operating_point(
        sim,
        phi_ref_rad=phi_ref,
        theta_ref_rad=theta_ref,
        settle_s=args.trim_settle_s,
        hold_s=args.trim_hold_s,
        dt=args.dt,
    )
    print(f"[trim] biases: {biases}")
    print(
        f"[trim] final:  phi={snap['phi']:.3f}, theta={snap['theta']:.3f}, beta={snap['beta']:.3f}, p={snap['p']:.3f}, q={snap['q']:.3f}, r={snap['r']:.3f}"
    )

    # 2) Build perturbations (sequential PRBS per axis)
    t, u_a, u_e, u_r = build_sequential_prbs(
        dt=args.dt,
        settle_after_trim_s=args.post_trim_settle_s,
        seg_s=args.seg_s,
        prbs_bit_s=args.prbs_bit_s,
        amp_a=args.amp_a,
        amp_e=args.amp_e,
        amp_r=args.amp_r,
        seed=args.seed,
    )

    # 3) Run + log
    run_perturbation_and_log(sim, biases, t, u_a, u_e, u_r, out_csv=args.out_csv)


if __name__ == "__main__":
    main()
