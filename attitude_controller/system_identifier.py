#!/usr/bin/env python3
"""
System ID for inner-loop (rate) models using JSBSim_Sandbox.

We identify first-order rate dynamics for:
  p_dot = a_p * p + b_p * delta_a
  q_dot = a_q * q + b_q * delta_e
  r_dot = a_r * r + b_r * delta_r

Then report:
  tau_p = -1/a_p,  k_phi_rate_to_int = b_p/(-a_p)
  tau_q = -1/a_q,  k_theta_rate_to_int = b_q/(-a_q)
  tau_r = -1/a_r

Outputs:
  - CSV log of the experiment
  - JSON with identified parameters
"""

import argparse
import json
import math
import os
import sys
from dataclasses import asdict

import numpy as np

# --- Import your sandbox + interfaces exactly like your app does ---
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import units
from vehicle_interface.vehicle_interface import ControlCommands, MockSensors  # noqa: E402

# If JSBSim_Sandbox is not in this file, import from where you placed it.
# Here we assume it's in the same repo path as shown in your snippet.
from jsbsim_sandbox.sandbox_sim import (  # <-- replace with the actual module path that defines JSBSim_Sandbox
    JSBSim_Sandbox,
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
)


# -------------------- helpers --------------------

def compute_beta_rad(fdm) -> float:
    """Compute sideslip β ≈ atan2(v, u) in body frame."""
    # JSBSim exposes u/v in ft/s; unit choice cancels in atan2
    u_fps = fdm["velocities/u-fps"]
    v_fps = fdm["velocities/v-fps"]
    return math.atan2(v_fps, u_fps)


def prbs_sequence(num_steps: int, bit_len: int, amplitude: float, seed: int = 1):
    """
    Generate a simple PRBS-like +/- amplitude sequence with piecewise-constant segments.

    num_steps : total samples
    bit_len   : samples per bit (segment length)
    amplitude : magnitude of +/- step
    """
    rng = np.random.default_rng(seed)
    num_bits = max(1, int(np.ceil(num_steps / bit_len)))
    bits = rng.choice([-1.0, 1.0], size=num_bits)
    u = np.repeat(bits, bit_len)[:num_steps]
    return amplitude * u


def fit_first_order_arx(y: np.ndarray, u: np.ndarray, dt: float):
    """
    Fit discrete ARX(1,0) with bias: y[k+1] = a_d y[k] + b_d u[k] + c_d
    Return (a_c, b_c, c_d) in continuous time using exact ZOH conversion.

    ZOH relations for x_dot = a_c x + b_c u:
      a_d = exp(a_c dt)
      b_d = \int_0^dt exp(a_c τ) dτ * b_c = ((exp(a_c dt) - 1)/a_c) * b_c
      => a_c = ln(a_d)/dt
         b_c = b_d * a_c / (exp(a_c dt) - 1)     (use b_c ≈ b_d/dt if a_c ~ 0)
    """
    assert y.ndim == 1 and u.ndim == 1 and len(y) == len(u)
    yk = y[:-1]
    yk1 = y[1:]
    uk = u[:-1]

    # Design matrix [y[k], u[k], 1] -> y[k+1]
    Phi = np.column_stack([yk, uk, np.ones_like(yk)])
    theta, *_ = np.linalg.lstsq(Phi, yk1, rcond=None)
    a_d, b_d, c_d = theta

    # Stabilize/log guard
    if a_d <= 0:
        # If noise yields non-physical a_d <= 0, clamp slightly positive
        a_d = max(a_d, 1e-6)

    a_c = math.log(a_d) / dt
    exp_ad = math.exp(a_c * dt)
    if abs(a_c) < 1e-6:
        b_c = b_d / dt
    else:
        b_c = b_d * a_c / (exp_ad - 1.0)

    return a_c, b_c, c_d, a_d, b_d


def summarize_rate_channel(name: str, a_c: float, b_c: float):
    """Return a small dict with nice derived quantities for a rate channel."""
    out = {
        "a_c": a_c,
        "b_c": b_c,
        "tau_s": None,
        "k_rate_DC": None,            # DC gain of u->rate (b_c / -a_c)
        "k_rate_to_integrator": None  # cascaded gain for angle plant: b_c / -a_c
    }
    if a_c < 0.0:
        out["tau_s"] = -1.0 / a_c
        out["k_rate_DC"] = b_c / (-a_c)
        out["k_rate_to_integrator"] = b_c / (-a_c)
    return out


# -------------------- main experiment --------------------

def run_sysid(
    aircraft: str,
    jsbsim_root: str,
    dt: float,
    settle_s: float,
    seg_s: float,
    aileron_amp: float,
    elevator_amp: float,
    rudder_amp: float,
    prbs_bit_s: float,
    duration_s: float,
    seed: int,
    out_dir: str,
):
    os.makedirs(out_dir, exist_ok=True)

    initial_cond = JSBSimVehicleInitialCond(
        h0_m=800.0,
        vt0_mps=40.0,
        lat0_deg=37.4275,
        lon0_deg=-122.1697,
        psi0_rad=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=dt)
    vehicle_config = JSBSimVehicleConfig(
        model_name=aircraft,
        root_dir=jsbsim_root,
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )
    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)

    # Build an experiment timeline:
    # [settle] + [aileron PRBS] + [elevator PRBS] + [rudder PRBS]
    n_settle = int(round(settle_s / dt))
    n_seg = int(round(seg_s / dt))
    n_total = n_settle + 3 * n_seg
    if duration_s is not None:
        n_total = min(n_total, int(round(duration_s / dt)))

    bit_len = max(1, int(round(prbs_bit_s / dt)))
    u_a = np.zeros(n_total)
    u_e = np.zeros(n_total)
    u_r = np.zeros(n_total)

    # Aileron segment
    if n_total > n_settle:
        nA = min(n_seg, n_total - n_settle)
        u_a[n_settle:n_settle + nA] = prbs_sequence(nA, bit_len, aileron_amp, seed + 1)

    # Elevator segment
    idx_E_start = n_settle + n_seg
    if n_total > idx_E_start:
        nE = min(n_seg, n_total - idx_E_start)
        u_e[idx_E_start:idx_E_start + nE] = prbs_sequence(nE, bit_len, elevator_amp, seed + 2)

    # Rudder segment
    idx_R_start = n_settle + 2 * n_seg
    if n_total > idx_R_start:
        nR = min(n_seg, n_total - idx_R_start)
        u_r[idx_R_start:idx_R_start + nR] = prbs_sequence(nR, bit_len, rudder_amp, seed + 3)

    # Storage
    t = np.arange(n_total) * dt
    phi = np.zeros(n_total)
    theta = np.zeros(n_total)
    p = np.zeros(n_total)
    q = np.zeros(n_total)
    r = np.zeros(n_total)
    beta = np.zeros(n_total)
    vt = np.zeros(n_total)
    h = np.zeros(n_total)

    # Controls (normalized) and run
    cc = ControlCommands(
        aileron_deflection_norm=0.0,
        elevator_deflection_norm=0.0,
        rudder_deflection_norm=0.0,
        spoiler_deflection=0.0,
    )

    for k in range(n_total):
        cc.aileron_deflection_norm = float(u_a[k])
        cc.elevator_deflection_norm = float(u_e[k])
        cc.rudder_deflection_norm = float(u_r[k])

        sensors = sim.step(cc)

        # Log states
        phi[k] = sim.fdm["attitude/phi-rad"]
        theta[k] = sim.fdm["attitude/theta-rad"]
        p[k] = sim.fdm["velocities/p-rad_sec"]
        q[k] = sim.fdm["velocities/q-rad_sec"]
        r[k] = sim.fdm["velocities/r-rad_sec"]
        beta[k] = compute_beta_rad(sim.fdm)
        vt[k] = units.knots_to_mps(sim.fdm["velocities/vtrue-kts"])
        h[k] = units.feet_to_meters(sim.fdm["position/h-sl-ft"])

    # Save raw log (CSV)
    csv_path = os.path.join(out_dir, "sysid_log.csv")
    header = "t,phi,theta,p,q,r,beta,u_a,u_e,u_r,vt,h"
    data = np.column_stack([t, phi, theta, p, q, r, beta, u_a, u_e, u_r, vt, h])
    np.savetxt(csv_path, data, delimiter=",", header=header, comments="")
    print(f"[sysid] wrote {csv_path}")

    # --- Fit per segment to reduce cross-coupling ---
    def seg_slice(start_idx, length):
        end = min(n_total, start_idx + length)
        return slice(start_idx, end)

    sl_settle = seg_slice(0, n_settle)
    sl_A = seg_slice(n_settle, n_seg)
    sl_E = seg_slice(n_settle + n_seg, n_seg)
    sl_R = seg_slice(n_settle + 2 * n_seg, n_seg)

    # Roll-rate model (aileron segment)
    a_p, b_p, cdp, a_d_p, b_d_p = fit_first_order_arx(p[sl_A], u_a[sl_A], dt)
    roll_summary = summarize_rate_channel("roll", a_p, b_p)

    # Pitch-rate model (elevator segment)
    a_q, b_q, cdq, a_d_q, b_d_q = fit_first_order_arx(q[sl_E], u_e[sl_E], dt)
    pitch_summary = summarize_rate_channel("pitch", a_q, b_q)

    # Yaw-rate model (rudder segment)
    a_r, b_r, cdr, a_d_r, b_d_r = fit_first_order_arx(r[sl_R], u_r[sl_R], dt)
    yaw_summary = summarize_rate_channel("yaw", a_r, b_r)

    # (Optional) cross-coupling quick looks (e.g., r vs aileron on A-segment)
    # r[k+1] = a_d*r[k] + b_da*u_a[k] + c_d  -> gives an estimate of N_{delta_a} effect
    _, b_r_from_da, _, _, _ = fit_first_order_arx(r[sl_A], u_a[sl_A], dt)
    _, b_p_from_dr, _, _, _ = fit_first_order_arx(p[sl_R], u_r[sl_R], dt)

    # JSON report
    report = {
        "meta": {
            "aircraft": aircraft,
            "dt_s": dt,
            "settle_s": settle_s,
            "segment_s": seg_s,
            "prbs_bit_s": prbs_bit_s,
            "amps": {"aileron": aileron_amp, "elevator": elevator_amp, "rudder": rudder_amp},
            "samples": int(n_total),
        },
        "roll_rate": roll_summary | {"a_d": a_d_p, "b_d": b_d_p, "bias_d": cdp},
        "pitch_rate": pitch_summary | {"a_d": a_d_q, "b_d": b_d_q, "bias_d": cdq},
        "yaw_rate": yaw_summary | {"a_d": a_d_r, "b_d": b_d_r, "bias_d": cdr},
        "cross_coupling_discrete": {
            "r_from_aileron_b_d": b_r_from_da,
            "p_from_rudder_b_d": b_p_from_dr,
        },
        "notes": [
            "k_rate_DC = b_c / (-a_c) is DC gain from control to rate.",
            "k_rate_to_integrator is the cascaded gain for angle plant G(s) ≈ [b_c] / [s (s - a_c)].",
            "Use tau_s and k_rate_to_integrator to seed PI+D angle controllers.",
        ],
    }

    json_path = os.path.join(out_dir, "sysid_report.json")
    with open(json_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"[sysid] wrote {json_path}")

    # Friendly console summary
    def fmt(s): return "None" if s is None else f"{s:.4g}"
    print("\n==== Identified rate models (continuous-time) ====")
    print(f"Roll:  p_dot = {fmt(a_p)} * p + {fmt(b_p)} * delta_a   "
          f"(tau_p={fmt(roll_summary['tau_s'])}, k_rate_to_int={fmt(roll_summary['k_rate_to_integrator'])})")
    print(f"Pitch: q_dot = {fmt(a_q)} * q + {fmt(b_q)} * delta_e   "
          f"(tau_q={fmt(pitch_summary['tau_s'])}, k_rate_to_int={fmt(pitch_summary['k_rate_to_integrator'])})")
    print(f"Yaw:   r_dot = {fmt(a_r)} * r + {fmt(b_r)} * delta_r   "
          f"(tau_r={fmt(yaw_summary['tau_s'])})")
    print("Cross-coupling (discrete quick-look): "
          f"r<-aileron b_d={fmt(b_r_from_da)}, p<-rudder b_d={fmt(b_p_from_dr)}")

    return report, csv_path, json_path


# -------------------- CLI --------------------

def main():
    p = argparse.ArgumentParser(description="Inner-loop system ID (rate models) for JSBSim glider.")
    p.add_argument("--aircraft", type=str, default="ask21", help="Aircraft model (e.g., 'glider')")
    p.add_argument("--jsbsim_root", type=str, default="jsbsim_sandbox/", help="JSBSim root (optional)")
    p.add_argument("--dt", type=float, default=0.01, help="Simulation step [s]")
    p.add_argument("--settle_s", type=float, default=3.0, help="Initial settle duration [s]")
    p.add_argument("--segment_s", type=float, default=20.0, help="Duration per excitation segment [s]")
    p.add_argument("--duration_s", type=float, default=None, help="Override total duration [s]")
    p.add_argument("--prbs_bit_s", type=float, default=0.5, help="PRBS bit length [s]")
    p.add_argument("--aileron_amp", type=float, default=0.05, help="Aileron normalized amplitude")
    p.add_argument("--elevator_amp", type=float, default=0.05, help="Elevator normalized amplitude")
    p.add_argument("--rudder_amp", type=float, default=0.05, help="Rudder normalized amplitude")
    p.add_argument("--seed", type=int, default=1, help="PRBS RNG seed")
    p.add_argument("--out_dir", type=str, default="output/sysid", help="Output directory")
    args = p.parse_args()

    run_sysid(
        aircraft=args.aircraft,
        jsbsim_root=args.jsbsim_root,
        dt=args.dt,
        settle_s=args.settle_s,
        seg_s=args.segment_s,
        aileron_amp=args.aileron_amp,
        elevator_amp=args.elevator_amp,
        rudder_amp=args.rudder_amp,
        prbs_bit_s=args.prbs_bit_s,
        duration_s=args.duration_s,
        seed=args.seed,
        out_dir=args.out_dir,
    )


if __name__ == "__main__":
    main()
