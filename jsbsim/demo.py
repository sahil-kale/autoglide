#!/usr/bin/env python3
"""
Quick JSBSim toy runner (glider) — minimal API demo.

Requirements:
  pip install jsbsim

Notes:
- Set JSBSIM_ROOT env var if jsbsim can’t find its data directory automatically.
- The script attempts to load a glider model from a small shortlist; adjust as needed.
"""

import os
import csv
import math
import argparse
from pathlib import Path
import jsbsim

# -------- Defaults (override with CLI flags) --------
SIM_TIME_SECONDS_DEFAULT = 60.0
DT_DEFAULT = 0.01  # 100 Hz is usually enough to capture main aero dynamics
INITIAL_HEIGHT_M_DEFAULT = 800.0
INITIAL_VEL_MPS_DEFAULT = 40.0
INITIAL_LAT_DEG_DEFAULT = 37.4275
INITIAL_LON_DEG_DEFAULT = -122.1697
INITIAL_HEADING_DEG_DEFAULT = 90.0  # east
OUTPUT_DIR_DEFAULT = "output/jsbsim/toy"

# Some common glider model names in JSBSim repos. Adjust/extend if needed.
GLIDER_CANDIDATES = [
    "ASK21",
    "ASW28",
    "DG808S",
    "zögling",  # sometimes appears as non-ASCII; won’t match, but we try anyway
    "Zogling",
    "sailplane",
]

def m_to_ft(m): return m * 3.280839895
def mps_to_kts(mps): return mps * 1.943844492
def fps_to_mps(fps): return fps * 0.3048

def try_load_glider(fdm, candidates):
    """Try model names in order; return the first that loads, else None."""
    for name in candidates:
        try:
            if fdm.load_model(name):
                return name
        except Exception:
            pass
    return None

def build_argparser():
    ap = argparse.ArgumentParser(description="Run a minimal JSBSim sim with a glider and log to CSV.")
    ap.add_argument("--sim-time", type=float, default=SIM_TIME_SECONDS_DEFAULT, help="Simulation duration in seconds")
    ap.add_argument("--dt", type=float, default=DT_DEFAULT, help="Fixed time step (seconds)")
    ap.add_argument("--h0-m", type=float, default=INITIAL_HEIGHT_M_DEFAULT, help="Initial altitude (meters MSL)")
    ap.add_argument("--vt0-mps", type=float, default=INITIAL_VEL_MPS_DEFAULT, help="Initial speed (m/s, approx TAS/CAS)")
    ap.add_argument("--lat0-deg", type=float, default=INITIAL_LAT_DEG_DEFAULT, help="Initial latitude (deg)")
    ap.add_argument("--lon0-deg", type=float, default=INITIAL_LON_DEG_DEFAULT, help="Initial longitude (deg)")
    ap.add_argument("--psi0-deg", type=float, default=INITIAL_HEADING_DEG_DEFAULT, help="Initial true heading (deg)")
    ap.add_argument("--output-dir", type=str, default=OUTPUT_DIR_DEFAULT, help="Directory to write the CSV log")
    ap.add_argument("--aircraft", type=str, default=None, help="Force a specific aircraft name (overrides glider search)")
    ap.add_argument("--root", type=str, default=os.environ.get("JSBSIM_ROOT", None),
                    help="JSBSim root directory (defaults to $JSBSIM_ROOT if set)")
    return ap

def main():
    args = build_argparser().parse_args()

    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_csv = out_dir / "run.csv"

    # Create FDM
    fdm = jsbsim.FGFDMExec(args.root)
    fdm.set_debug_level(0)

    # Load aircraft (prefer glider)
    if args.aircraft:
        ok = fdm.load_model(args.aircraft)
        if not ok:
            raise RuntimeError(f"Could not load requested aircraft '{args.aircraft}'. "
                               f"Check your JSBSIM_ROOT and model name.")
        model_name = args.aircraft
    else:
        model_name = try_load_glider(fdm, GLIDER_CANDIDATES)
        if model_name is None:
            raise RuntimeError(
                "Could not load any glider model from candidates. "
                "Try passing --aircraft <name> and ensure your JSBSIM_ROOT is correct."
            )

    # Initialize: set ICs through 'ic/*' properties; run_ic() applies them
    fdm.set_property_value("ic/h-sl-ft", m_to_ft(args.h0_m))
    # Use calibrated airspeed (CAS) or true airspeed (vt). CAS is more standard in ic:
    fdm.set_property_value("ic/vc-kts", mps_to_kts(args.vt0_mps))
    fdm.set_property_value("ic/psi-true-deg", args.psi0_deg)
    fdm.set_property_value("ic/lat-gc-deg", args.lat0_deg)
    fdm.set_property_value("ic/long-gc-deg", args.lon0_deg)

    # Trim initial attitude to something reasonable (optional; many gliders start level)
    # If your model needs explicit pitch/roll/yaw set, uncomment:
    # fdm.set_property_value("ic/theta-deg", 0.0)  # pitch
    # fdm.set_property_value("ic/phi-deg", 0.0)    # roll
    # fdm.set_property_value("ic/psi-true-deg", args.psi0_deg)  # yaw/heading already set

    # Apply the initial condition
    if not fdm.run_ic():
        raise RuntimeError("run_ic() failed; check IC properties and model availability.")

    # Use fixed time step
    dt = float(args.dt)
    fdm.set_dt(dt)

    # Example: neutralize primary controls (glider; aileron/elevator/rudder)
    # These are normalized [-1, 1]. Adjust if needed.
    # (If a given aircraft uses different control names, these calls are harmless no-ops.)
    fdm.set_property_value("fcs/aileron-cmd-norm", 0.0)
    fdm.set_property_value("fcs/elevator-cmd-norm", -0.151)
    fdm.set_property_value("fcs/rudder-cmd-norm", 0.0)
    # Spoiler as an example instead of throttle (gliders): keep retracted
    fdm.set_property_value("fcs/speedbrake-cmd-norm", 0.0)

    # Write a tiny CSV log ourselves (time, pos, attitude, speed, etc.)
    fields = [
        "time_s",
        "lat_deg", "lon_deg",
        "alt_msl_m",
        "phi_roll_deg", "theta_pitch_deg", "psi_hdg_deg",
        "vt_mps", "tas_mps",
        "alpha_deg", "beta_deg",
        "p_roll_rate_dps", "q_pitch_rate_dps", "r_yaw_rate_dps",
        "nx_g", "ny_g", "nz_g",
    ]

    with out_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        sim_time = 0.0
        t_end = float(args.sim_time)

        while sim_time < t_end:
            # Advance the simulation one step
            if not fdm.run():
                raise RuntimeError("fdm.run() failed mid-simulation.")

            # Collect a few common properties (robust to missing keys by using defaults)
            def g(prop, default=0.0):
                try:
                    return fdm.get_property_value(prop)
                except Exception:
                    return default

            row = {
                "time_s": g("simulation/sim-time-sec"),
                "lat_deg": g("position/lat-gc-deg"),
                "lon_deg": g("position/long-gc-deg"),
                "alt_msl_m": g("position/h-sl-meters"),
                "phi_roll_deg": g("attitude/phi-deg"),
                "theta_pitch_deg": g("attitude/theta-deg"),
                "psi_hdg_deg": g("attitude/psi-deg"),
                "vt_mps": g("velocities/vc-mps"),  # calibrated airspeed (approx)
                "tas_mps": g("velocities/vtrue-kts") * 0.514444,  # true airspeed
                "alpha_deg": g("aerodynamics/alpha-deg"),
                "beta_deg": g("aerodynamics/beta-deg"),
                "p_roll_rate_dps": g("velocities/p-aero-degps"),
                "q_pitch_rate_dps": g("velocities/q-aero-degps"),
                "r_yaw_rate_dps": g("velocities/r-aero-degps"),
                "nx_g": g("accelerations/Nx"),
                "ny_g": g("accelerations/Ny"),
                "nz_g": g("accelerations/Nz"),
            }
            writer.writerow(row)

            sim_time += dt

    print(f"[OK] Model: {model_name}")
    print(f"[OK] Simulated {args.sim_time:.2f}s at dt={dt:.4f}s -> {int(args.sim_time/dt)} steps")
    print(f"[OK] CSV written: {out_csv.resolve()}")

if __name__ == "__main__":
    main()
