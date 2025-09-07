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
from dataclasses import dataclass
from typing import List

import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D projection)
from mpl_toolkits.mplot3d.art3d import Line3DCollection

# Ensure package root is on path (one level up from this file)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from glider_model.model import (  # noqa: E402
    GliderModelParams,
    GliderKinematicModelControl,
    GliderKinematicModelDisturbance,
    GliderOpenLoopKinematicModel,
)

# --- Matplotlib keymap overrides (disable default bindings that conflict with controls) ---
mpl.rcParams["keymap.save"] = []       # 's'
mpl.rcParams["keymap.fullscreen"] = [] # 'f'
mpl.rcParams["keymap.pan"] = []        # 'p'
mpl.rcParams["keymap.zoom"] = []       # 'o', etc.

# --- Simulation/Control constants ---
SIM_DT: float = 0.1
ROLL_STEP_RAD: float = np.deg2rad(5.0)
VEL_STEP_MS: float = 1.0
ROLL_LIMIT_RAD: float = np.pi / 2
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
    def __init__(self, params: GliderModelParams, dt: float = SIM_DT) -> None:
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

        self.xs: List[float] = [self.glider.x]
        self.ys: List[float] = [self.glider.y]
        self.hs: List[float] = [self.glider.h]

        self.fig = plt.figure(figsize=(10, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")
        plt.subplots_adjust(bottom=0.2)
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        self._step_count = 0
        self._time = 0.0

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

        # Hook for thermal model: set disturbance.w = thermal.get_thermal_uplift(...)
        self.disturbance.w = 0.0

        self.glider.step(self.dt, self.control, self.disturbance)

        self.xs.append(self.glider.x)
        self.ys.append(self.glider.y)
        self.hs.append(self.glider.h)

        self._time += self.dt
        self._step_count += 1

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

    def draw(self) -> None:
        self.ax.clear()

        self.ax.plot(self.xs, self.ys, self.hs, linewidth=2, label="Path")

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

        plt.pause(0.001)

    def run(self) -> None:
        try:
            while True:
                self.step()
                if self._step_count % DRAW_EVERY_STEPS == 0:
                    self.draw()
        except KeyboardInterrupt:
            pass


def main() -> None:
    params = GliderModelParams(
        V_star=21.11,       # m/s
        s_min=0.68,         # m/s
        k_v=0.0031,         # (m/s)/(m/s)^2
        alpha_n=1.3,        # -
        initial_altitude=300.0,  # m
        roll_tau=1.0,       # s
        vel_tau=2.0,        # s
    )
    sim = SingleThermalGliderSimulator(params=params, dt=SIM_DT)
    sim.run()


if __name__ == "__main__":
    main()
