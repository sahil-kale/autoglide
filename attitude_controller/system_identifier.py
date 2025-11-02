import os
import sys
import csv
from dataclasses import dataclass

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_interface import vehicle_interface  # not used here, but kept if your repo expects it
from jsbsim_sandbox.sandbox_sim import *         # uses JSBSim_Sandbox & config classes
import numpy as np
import jsbsim


def _clamp(x, lo=-1.0, hi=1.0):
    return max(lo, min(hi, x))


@dataclass
class TrimGains:
    # Tiny PI gains to nudge the sim toward a steady state
    kq: float = 0.25     # pitch-rate -> elevator P
    kiq: float = 0.05    # pitch-rate -> elevator I
    kp: float = 0.20     # roll-rate  -> aileron P
    kip: float = 0.05    # roll-rate  -> aileron I
    kb: float = 0.20     # beta       -> rudder  P
    kib: float = 0.05    # beta       -> rudder  I
    kr: float = 0.20     # small yaw-damper on r


class SystemIdentifierSingleCase:
    def __init__(self, sim: JSBSim_Sandbox, trim_gains: TrimGains | None = None):
        self.sim = sim
        self.g = trim_gains or TrimGains()

        # convenience alias
        self.fdm = self.sim.fdm
        self.dt = self.sim.sim_params.dt_s

        # where to write logs
        self.out_dir = os.path.join("output", "jsbsim", "toy")
        os.makedirs(self.out_dir, exist_ok=True)
        self.csv_path = os.path.join(self.out_dir, "system_id_log.csv")

    # ---- Numeric "trim" that always works for gliders ----
    def _numeric_trim(self, duration_s: float = 6.0, sink_fps: float | None = None):
        """
        Runs a short PI loop on (q,p,beta,r) to converge to a quasi-steady flight condition.
        If sink_fps is provided (negative value), biases elevator slightly to target that sink rate.
        """
        dt = self.dt
        steps = int(duration_s / dt)

        # Initialize surface commands near zero
        self.fdm['fcs/elevator-cmd-norm'] = 0.0
        self.fdm['fcs/aileron-cmd-norm']  = 0.0
        self.fdm['fcs/rudder-cmd-norm']   = 0.0

        iq = ip = ib = 0.0

        # tiny bias to pursue a desired sink if requested
        k_vz_bias = 0.0
        if sink_fps is not None:
            k_vz_bias = 0.001  # very small

        for _ in range(steps):
            self.fdm.run()

            q = float(self.fdm['velocities/q-rad_sec'])
            p = float(self.fdm['velocities/p-rad_sec'])
            r = float(self.fdm['velocities/r-rad_sec'])
            beta = float(self.fdm['aero/beta-rad'])
            vz_fps = float(self.fdm['velocities/h-dot-fps'])  # +up

            # Integrators
            iq += (-q) * dt
            ip += (-p) * dt
            ib += (-beta) * dt

            # Commands (P + I + small yaw damper + optional sink bias)
            de = self.g.kq * (-q) + self.g.kiq * iq + k_vz_bias * (0.0 if sink_fps is None else (sink_fps - vz_fps))
            da = self.g.kp * (-p) + self.g.kip * ip
            dr = self.g.kb * (-beta) + self.g.kib * ib + self.g.kr * (-r)

            # Apply with saturation
            self.fdm['fcs/elevator-cmd-norm'] = _clamp(de)
            self.fdm['fcs/aileron-cmd-norm']  = _clamp(da)
            self.fdm['fcs/rudder-cmd-norm']   = _clamp(dr)

        # Return trim values
        return (
            float(self.fdm['fcs/elevator-cmd-norm']),
            float(self.fdm['fcs/aileron-cmd-norm']),
            float(self.fdm['fcs/rudder-cmd-norm']),
        )

    def _write_csv_header(self, writer: csv.writer):
        writer.writerow([
            "t",
            "theta_rad", "phi_rad",
            "q_radps", "p_radps", "r_radps",
            "beta_rad",
            "vt_mps", "h_dot_mps",
            "de_cmd", "da_cmd", "dr_cmd"
        ])

    def _log_row(self, writer: csv.writer):
        writer.writerow([
            float(self.fdm.get_sim_time()),
            float(self.fdm["attitude/theta-rad"]),
            float(self.fdm["attitude/phi-rad"]),
            float(self.fdm["velocities/q-rad_sec"]),
            float(self.fdm["velocities/p-rad_sec"]),
            float(self.fdm["velocities/r-rad_sec"]),
            float(self.fdm["aero/beta-rad"]),
            float(self.fdm["velocities/vtrue-kts"]) * 0.514444,
            float(self.fdm["velocities/h-dot-fps"]) * 0.3048,
            float(self.fdm['fcs/elevator-cmd-norm']),
            float(self.fdm['fcs/aileron-cmd-norm']),
            float(self.fdm['fcs/rudder-cmd-norm']),
        ])

    def run(self, stop_time_s: float):
        """
        1) Perform a short numeric trim (robust for gliders).
        2) Run sim to stop_time_s.
        3) Inject two tiny pulses (elevator then aileron) for quick SI.
        4) Log to CSV for later identification.
        """
        # --- Step 1: numeric trim (for gliders, request a small sink so it's physically consistent)
        elev_trim, ail_trim, rud_trim = self._numeric_trim(duration_s=6.0, sink_fps=-2.6)  # ~ -0.8 m/s

        # Hold the trim values as baseline
        self.fdm['fcs/elevator-cmd-norm'] = elev_trim
        self.fdm['fcs/aileron-cmd-norm']  = ail_trim
        self.fdm['fcs/rudder-cmd-norm']   = rud_trim

        # --- Step 2: define simple test inputs (optional)
        # Elevator pulse: +0.02 for 0.10 s starting at t=80.0 s
        elev_pulse_t0 = 80.0
        elev_pulse_dt = 1.0
        elev_pulse_du = 1.0

        # Aileron pulse: +0.02 for 0.10 s starting at t=100.0 s
        ail_pulse_t0 = 10.0
        ail_pulse_dt = 1.0
        ail_pulse_du = 1.0

        # --- Step 3: run and log
        with open(self.csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            self._write_csv_header(writer)

            t = float(self.fdm.get_sim_time())
            while t < stop_time_s:
                # Default to trim
                de_cmd = elev_trim
                da_cmd = ail_trim
                dr_cmd = rud_trim

                # Apply test pulses (time-boxed & small)
                if elev_pulse_t0 <= t < (elev_pulse_t0 + elev_pulse_dt):
                    de_cmd = _clamp(elev_trim + elev_pulse_du)
                if ail_pulse_t0 <= t < (ail_pulse_t0 + ail_pulse_dt):
                    da_cmd = _clamp(ail_trim + ail_pulse_du)

                self.fdm['fcs/elevator-cmd-norm'] = de_cmd
                self.fdm['fcs/aileron-cmd-norm']  = da_cmd
                self.fdm['fcs/rudder-cmd-norm']   = dr_cmd

                self.fdm.run()
                self._log_row(writer)
                t = float(self.fdm.get_sim_time())

        print("[system_id] Done.")
        print(f"[system_id] Trim (de, da, dr): {elev_trim:.4f}, {ail_trim:.4f}, {rud_trim:.4f}")
        print(f"[system_id] Log written to: {self.csv_path}")


if __name__ == "__main__":
    # for now, just straight and level-ish (glider sinks slightly)
    initial_cond = JSBSimVehicleInitialCond(
        h0_m=800.0,
        vt0_mps=40.0,
        lat0_deg=37.4275,
        lon0_deg=-122.1697,
        psi0_rad=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=0.01)
    vehicle_config = JSBSimVehicleConfig(
        model_name="ask21",
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
        root_dir="jsbsim_sandbox/"
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)
    identifier = SystemIdentifierSingleCase(sim)
    identifier.run(stop_time_s=120.0)
