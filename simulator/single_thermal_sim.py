from __future__ import annotations


import os
import sys
import json
from dataclasses import dataclass
from typing import List
import argparse
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
from thermal_estimator.thermal_estimator import ThermalEstimator
from utils.location import WorldFrameCoordinate
from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState
from simulator.utils.airplane_glyph import draw_airplane


# --- Matplotlib keymap overrides (disable default bindings that conflict with controls) ---
mpl.rcParams["keymap.save"] = []  # 's'
mpl.rcParams["keymap.fullscreen"] = []  # 'f'
mpl.rcParams["keymap.pan"] = []  # 'p'
mpl.rcParams["keymap.zoom"] = []  # 'o', etc.

# --- Simulation/Control constants ---
SIM_DT: float = 0.1
ROLL_STEP_RAD: float = np.deg2rad(5.0)
VEL_STEP_MS: float = 1.0
ROLL_LIMIT_RAD: float = np.deg2rad(60.0)
V_MIN: float = 20.0
V_MAX: float = 50.0
PLOT_SPAN_XY: float = 100.0
PLOT_SPAN_Z: float = 50.0
DRAW_EVERY_STEPS: int = 2
AIRPLANE_SCALE: float = 6.0
# --- Estimator/Plotting ---
PLOT_ESTIMATED_THERMAL_PARAMS = True

# --- Oscilloscope plot settings ---
SCOPE_WINDOW_SEC: float = 30.0  # seconds to show in scope plots
SCOPE_UPDATE_EVERY: int = 10  # update scope plots every N frames


@dataclass
class ControlState:
    roll_rad: float
    airspeed_ms: float


class SingleThermalGliderSimulator:
    def __init__(
        self,
        params: GliderModelParams,
        dt: float = SIM_DT,
        thermal: ThermalModel = None,
        manual_mode: bool = False,
    ) -> None:
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

        # Vehicle state estimator
        from vehicle_state_estimator.vehicle_state_estimator import (
            VehicleState,
            VehicleStateEstimatorPassthrough,
        )
        from utils.vector import Vector2D

        initial_state = VehicleState(
            position=WorldFrameCoordinate(self.glider.x, self.glider.y),
            airspeed=self.glider.V,
            velocity_ground=Vector2D(0.0, 0.0),
            heading=self.glider.psi,
        )
        self.vehicle_state_estimator = VehicleStateEstimatorPassthrough(initial_state)

        # Scalable scope data structure: {name: [values]}
        self.scope_data = {
            "Altitude (m)": [self.glider.h],
            "Airspeed (m/s)": [self.glider.V],
            "Roll (deg)": [np.rad2deg(self.glider.phi)],
            "Uplift Speed (m/s)": [0.0],
            "Estimator Confidence": [1.0],
            "Guidance State": ["Cruise"],
        }

        self.fig = plt.figure(figsize=(16, 8))
        gs = gridspec.GridSpec(
            6, 2, width_ratios=[2, 1], height_ratios=[1, 1, 1, 1, 1, 1]
        )
        self.ax = self.fig.add_subplot(gs[:, 0], projection="3d")
        self.scope_axes = []
        scope_labels = list(self.scope_data.keys())
        for i in range(len(scope_labels)):
            ax = self.fig.add_subplot(gs[i, 1])
            ax.set_ylabel(scope_labels[i])
            self.scope_axes.append(ax)
        self.scope_axes[-1].set_xlabel("Time (s)")
        self.fig.tight_layout()
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

        # --- Estimator ---
        self.thermal_estimator = ThermalEstimator(num_samples_to_buffer=50)
        self._step_count = 0
        self._time = 0.0
        self.thermal = thermal

        # --- Guidance State Machine ---
        self.guidance_sm = GuidanceStateMachine(
            thermal_confidence_probe_threshold=0.3,
            thermal_confidence_circle_threshold=0.5,
            glider_model_params=params,
        )
        self.manual_mode = manual_mode

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
        # --- Glider step ---
        if self.thermal is not None:
            uplift = self.thermal.get_thermal_uplift(
                self.glider.x, self.glider.y, self.glider.h
            )
        else:
            uplift = 0.0

        self.disturbance.w = uplift

        # Use glider position as measurement location, and measured uplift as measurement
        location = WorldFrameCoordinate(self.glider.x, self.glider.y)
        self.thermal_estimator.step(uplift, location)
        thermal_estimate = self.thermal_estimator.get_estimate()

        # Update vehicle state estimator
        self.vehicle_state_estimator.update(
            glider_position=location,
            true_airspeed=self.glider.V,
            true_heading=self.glider.psi,
            dt=self.dt,
        )
        vehicle_state = self.vehicle_state_estimator.get_state()

        # Guidance state machine integration
        origin_wp = WorldFrameCoordinate(0.0, 0.0)
        target_wp = WorldFrameCoordinate(1000.0, 0.0)

        # Control Law Implementation
        control = self.guidance_sm.step(
            vehicle_state, thermal_estimate, origin_wp, target_wp
        )
        self.control.phi = control.phi
        self.control.V = control.V
        guidance_state_str = str(self.guidance_sm.get_state())

        if self.manual_mode:
            # Manual mode: use WASD controls
            self.control.phi = self.ctrl_state.roll_rad
            self.control.V = self.ctrl_state.airspeed_ms

        self.glider.step(self.dt, self.control, self.disturbance)

        self.xs.append(self.glider.x)
        self.ys.append(self.glider.y)
        self.hs.append(self.glider.h)

        self._time += self.dt
        self._step_count += 1

        # Store for time-domain plots
        self.times.append(self._time)
        self.scope_data["Altitude (m)"].append(self.glider.h)
        self.scope_data["Airspeed (m/s)"].append(self.glider.V)
        self.scope_data["Roll (deg)"].append(np.rad2deg(self.glider.phi))
        self.scope_data["Uplift Speed (m/s)"].append(uplift)
        self.scope_data["Estimator Confidence"].append(thermal_estimate.confidence)
        self.scope_data["Guidance State"].append(guidance_state_str)

    # --- Drawing ---

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
            self.ax.plot(
                ring_x, ring_y, ring_z, "b--", linewidth=1.5, label="Thermal Core"
            )

            # Optionally, plot core drift path (projected)
            hs_drift = np.linspace(self.glider.h - 100, self.glider.h + 100, 20)
            drift_x = []
            drift_y = []
            for h in hs_drift:
                xc, yc = self.thermal.core_center_at_height(h)
                drift_x.append(xc)
                drift_y.append(yc)
            self.ax.plot(
                drift_x, drift_y, hs_drift, "b:", linewidth=1, label="Core Drift Path"
            )

        # Plot estimated thermal center and radius
        if PLOT_ESTIMATED_THERMAL_PARAMS:
            thermal_estimate = self.thermal_estimator.get_estimate()
            est_core = thermal_estimate.get_location()
            est_Rth = thermal_estimate.get_radius()
            theta = np.linspace(0, 2 * np.pi, 100)
            est_ring_x = est_core.x + est_Rth * np.cos(theta)
            est_ring_y = est_core.y + est_Rth * np.sin(theta)
            est_ring_z = np.full_like(theta, self.glider.h)
            self.ax.plot(
                est_ring_x,
                est_ring_y,
                est_ring_z,
                "r--",
                linewidth=2,
                label="Estimated Core",
            )
            self.ax.scatter(
                est_core.x,
                est_core.y,
                self.glider.h,
                color="red",
                marker="x",
                s=80,
                label="Est Center",
            )

        draw_airplane(
            self.ax,
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
            # Only plot last SCOPE_WINDOW_SEC seconds
            t_window = SCOPE_WINDOW_SEC
            t_now = self._time
            times_arr = np.array(self.times)
            idx_start = np.searchsorted(times_arr, t_now - t_window)
            scope_items = list(self.scope_data.items())
            for ax, (label, data) in zip(self.scope_axes, scope_items):
                ax.clear()
                ax.plot(times_arr[idx_start:], np.array(data)[idx_start:])
                ax.set_ylabel(label)
            self.scope_axes[-1].set_xlabel("Time (s)")

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
    with open(
        os.path.join(os.path.dirname(__file__), "../ask21_kinematic_model.json"), "r"
    ) as f:
        model_params = json.load(f)

    params = GliderModelParams(
        V_star=model_params["V_star"],
        V_stall=model_params["V_stall"],
        s_min=model_params["s_min"],
        k_v=model_params["k_v"],
        alpha_n=model_params["alpha_n"],
        initial_altitude=300.0,  # m
        roll_tau=0.2,  # s
        vel_tau=0.2,  # s
    )

    # Example thermal model parameters (can be tuned or loaded from config)
    thermal = ThermalModel(
        w_max=7.0,
        r_th=70.0,
        x_th=100.0,
        y_th=50.0,
        V_e=1.0,
        kx=0.03,
        ky=-0.02,
        core_center_random_noise_std=1.5,
    )

    parser = argparse.ArgumentParser(description="Single Thermal Glider Simulator")
    parser.add_argument("--manual", action="store_true", help="Enable manual WASD mode")
    args = parser.parse_args()

    sim = SingleThermalGliderSimulator(
        params=params, dt=SIM_DT, thermal=thermal, manual_mode=args.manual
    )
    sim.run()


if __name__ == "__main__":
    main()
