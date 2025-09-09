"""
NOTE: This is vibe-coded - don't take it too seriously
I'm a controls guy, not a graphics programmer ;)

Single Thermal Glider Simulator (interactive)
- Open-loop kinematic glider stepped at fixed dt
- 3D visualization with WASD controls (A/D = roll, W/S = airspeed)
"""

from __future__ import annotations


import os
import sys
import json
from dataclasses import dataclass
from typing import List

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Ensure package root is on path (one level up from this file)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from glider_model.model import (
    GliderModelParams,
    GliderKinematicModelControl,
    GliderKinematicModelDisturbance,
    GliderOpenLoopKinematicModel,
)
from thermal_model.thermal_model import ThermalModel

# --- Matplotlib keymap overrides (disable default bindings that conflict with controls) ---
mpl.rcParams["keymap.save"] = []       # 's'
mpl.rcParams["keymap.fullscreen"] = [] # 'f'
mpl.rcParams["keymap.pan"] = []        # 'p'
mpl.rcParams["keymap.zoom"] = []       # 'o', etc.

# --- Simulation/Control constants ---
SIM_DT: float = 0.1
ROLL_STEP_RAD: float = np.deg2rad(5.0)
VEL_STEP_MS: float = 1.0
ROLL_LIMIT_RAD: float = np.pi / 4
V_MIN: float = 5.0
V_MAX: float = 50.0
PLOT_SPAN_XY: float = 100.0
PLOT_SPAN_Z: float = 50.0
DRAW_EVERY_STEPS: int = 2
AIRPLANE_SCALE: float = 6.0


@dataclass
class ControlState:
    roll_rad: float
    airspeed_ms: float



class SingleThermalGliderSimulator:
    def __init__(self, params: GliderModelParams, dt: float = SIM_DT, thermal: ThermalModel = None) -> None:
        self.params = params
        self.dt = dt

        self.glider = GliderOpenLoopKinematicModel(params)
        self.control = GliderKinematicModelControl(
            roll_angle=0.0, airspeed=params.V_star
        )
        self.disturbance = GliderKinematicModelDisturbance(
            thermal_uplift=0.0, wind_x=0.0, wind_y=0.0
        )
        self.ctrl_state = ControlState(roll_rad=0.0, airspeed_ms=params.V_star)

        self.xs = [self.glider.x]
        self.ys = [self.glider.y]
        self.hs = [self.glider.h]
        self.times = [0.0]
        self.Vs = [self.glider.V]
        self.phis = [self.glider.phi]
        self.uplifts = [0.0]

        self.fig = plt.figure(figsize=(16, 8))
        gs = gridspec.GridSpec(4, 2, width_ratios=[2, 1], height_ratios=[1, 1, 1, 1])
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.ax_alt = self.fig.add_subplot(gs[0, 1])
        self.ax_vel = self.fig.add_subplot(gs[1, 1])
        self.ax_roll = self.fig.add_subplot(gs[2, 1])
        self.ax_uplift = self.fig.add_subplot(gs[3, 1])
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._step_count = 0
        self._time = 0.0

        self.thermal = thermal

    # --- Event handling ---

    def _on_key(self, event) -> None:
        k = event.key
        if k == "a":
            self.ctrl_state.roll_rad += ROLL_STEP_RAD
        elif k == "d":
            self.ctrl_state.roll_rad -= ROLL_STEP_RAD
        elif k == "w":
            self.ctrl_state.airspeed_ms += VEL_STEP_MS
        elif k == "s":
            self.ctrl_state.airspeed_ms -= VEL_STEP_MS

        self.ctrl_state.roll_rad = float(
            np.clip(self.ctrl_state.roll_rad, -ROLL_LIMIT_RAD, ROLL_LIMIT_RAD)
        )
        self.ctrl_state.airspeed_ms = float(
            np.clip(self.ctrl_state.airspeed_ms, V_MIN, V_MAX)
        )

        print(
            f"Control: roll={np.rad2deg(self.ctrl_state.roll_rad):.1f} deg, "
            f"V={self.ctrl_state.airspeed_ms:.1f} m/s"
        )

    # --- Sim loop ---

    def step(self) -> None:
        self.control.phi = self.ctrl_state.roll_rad
        self.control.V = self.ctrl_state.airspeed_ms

        # Use thermal model if available
        if self.thermal is not None:
            uplift = self.thermal.get_thermal_uplift(self.glider.x, self.glider.y, self.glider.h)
            self.disturbance.w = uplift
        else:
            uplift = 0.0
            self.disturbance.w = 0.0

        self.glider.step(self.dt, self.control, self.disturbance)

        self.xs.append(self.glider.x)
        self.ys.append(self.glider.y)
        self.hs.append(self.glider.h)

        self._time += self.dt
        self._step_count += 1

        # Store for time-domain plots
        self.times.append(self._time)
        self.Vs.append(self.glider.V)
        self.phis.append(np.rad2deg(self.glider.phi))
        self.uplifts.append(uplift)

    # --- Drawing ---

    @staticmethod
    def _airplane_segments(x: float, y: float, z: float, psi: float, phi: float, scale: float):
        cpsi, spsi = np.cos(psi), np.sin(psi)
        ex = np.array([cpsi, spsi, 0.0])     # forward (body x) in world
        ey = np.array([-spsi, cpsi, 0.0])    # right (body y) in world
        ez = np.array([0.0, 0.0, 1.0])       # up (body z) in world

        p0 = np.array([x, y, z])

        nose   = p0 + ex * (scale * 0.8)
        tail   = p0 - ex * (scale * 0.7)

        # Wings tilted by roll: +/- z offset proportional to tan(phi)
        wz = np.tan(phi) * scale * 0.5
        wing_r = p0 + ey * (scale * 0.6) - ez * wz
        wing_l = p0 - ey * (scale * 0.6) + ez * wz

        tail_r = tail + ey * (scale * 0.25)
        tail_l = tail - ey * (scale * 0.25)

        segs = [
            (tail, nose),       # fuselage
            (wing_l, wing_r),   # wings
            (tail_l, tail_r),   # horizontal tail
        ]
        return segs, nose, ex, ey, ez

    def _draw_airplane(self, x: float, y: float, z: float, psi: float, phi: float, scale: float) -> None:
        segs, nose, ex, ey, ez = self._airplane_segments(x, y, z, psi, phi, scale)
        lc = Line3DCollection(segs, linewidths=2.0, colors="red")
        self.ax.add_collection3d(lc)

        # small nose arrow for direction cue
        self.ax.plot(
            [nose[0], nose[0] - 0.2 * ex[0] + 0.15 * ey[0], nose[0] - 0.2 * ex[0] - 0.15 * ey[0], nose[0]],
            [nose[1], nose[1] - 0.2 * ex[1] + 0.15 * ey[1], nose[1] - 0.2 * ex[1] - 0.15 * ey[1], nose[1]],
            [nose[2], nose[2] - 0.2 * ex[2] + 0.15 * ey[2], nose[2] - 0.2 * ex[2] - 0.15 * ey[2], nose[2]],
            linewidth=1.2,
        )

    def draw(self, update_oscopes: bool = True) -> None:
        self.ax.clear()

        self.ax.plot(self.xs, self.ys, self.hs, linewidth=2, label="Path")

        # Draw thermal core ring at current altitude
        if self.thermal is not None:
            x_c, y_c = self.thermal.core_center_at_height(self.glider.h)
            r_th = self.thermal.r_th
            theta = np.linspace(0, 2 * np.pi, 100)
            ring_x = x_c + r_th * np.cos(theta)
            ring_y = y_c + r_th * np.sin(theta)
            ring_z = np.full_like(theta, self.glider.h)
            self.ax.plot(ring_x, ring_y, ring_z, 'b--', linewidth=1.5, label="Thermal Core")

            # Optionally, plot core drift path (projected)
            hs_drift = np.linspace(self.glider.h - 100, self.glider.h + 100, 20)
            drift_x = []
            drift_y = []
            for h in hs_drift:
                xc, yc = self.thermal.core_center_at_height(h)
                drift_x.append(xc)
                drift_y.append(yc)
            self.ax.plot(drift_x, drift_y, hs_drift, 'b:', linewidth=1, label="Core Drift Path")

        # Airplane glyph at current pose
        self._draw_airplane(
            x=self.glider.x,
            y=self.glider.y,
            z=self.glider.h,
            psi=self.glider.psi,
            phi=self.glider.phi,
            scale=AIRPLANE_SCALE,
        )

        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Y (m)")
        self.ax.set_zlabel("Altitude (m)")
        self.ax.set_title(f"Glider Simulation (t={self._time:.1f}s)")
        self.ax.legend()

        self.ax.set_xlim(self.glider.x - PLOT_SPAN_XY, self.glider.x + PLOT_SPAN_XY)
        self.ax.set_ylim(self.glider.y - PLOT_SPAN_XY, self.glider.y + PLOT_SPAN_XY)
        self.ax.set_zlim(self.glider.h - PLOT_SPAN_Z, self.glider.h + PLOT_SPAN_Z)

        if update_oscopes:
            self.ax_alt.clear()
            self.ax_vel.clear()
            self.ax_roll.clear()
            self.ax_uplift.clear()
            self.ax_alt.plot(self.times, self.hs, color='g')
            self.ax_alt.set_ylabel("Altitude (m)")
            self.ax_vel.plot(self.times, self.Vs, color='b')
            self.ax_vel.set_ylabel("Airspeed (m/s)")
            self.ax_roll.plot(self.times, self.phis, color='r')
            self.ax_roll.set_ylabel("Roll (deg)")
            self.ax_uplift.plot(self.times, self.uplifts, color='m')
            self.ax_uplift.set_ylabel("Thermal Uplift (m/s)")
            self.ax_uplift.set_xlabel("Time (s)")

        plt.pause(0.001)

    def run(self) -> None:
        try:
            while True:
                self.step()
                # Always update 3D plot for smoothness
                self.draw(update_oscopes=(self._step_count % 10 == 0))
        except KeyboardInterrupt:
            pass



def main() -> None:
    # Load kinematic model parameters from JSON
    with open(os.path.join(os.path.dirname(__file__), "../ask21_kinematic_model.json"), "r") as f:
        model_params = json.load(f)

    params = GliderModelParams(
        V_star=model_params["V_star"],
        s_min=model_params["s_min"],
        k_v=model_params["k_v"],
        alpha_n=model_params["alpha_n"],
        initial_altitude=300.0,  # m
        roll_tau=1.0,            # s
        vel_tau=2.0,             # s
    )

    # Example thermal model parameters (can be tuned or loaded from config)
    thermal = ThermalModel(
        w_max=5.0,
        r_th=50.0,
        x_th=500.0,
        y_th=0.0,
        V_e=1.0,
        kx=0.03,
        ky=-0.02,
        core_center_random_noise_std=1.5,
    )

    sim = SingleThermalGliderSimulator(params=params, dt=SIM_DT, thermal=thermal)
    sim.run()


if __name__ == "__main__":
    main()
