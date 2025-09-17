from __future__ import annotations


import os
import sys
import json
from dataclasses import dataclass
from typing import List
import argparse
import numpy as np
import matplotlib as mpl
import numpy as np

# Ensure package root is on path (one level up from this file)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from glider_model.model import (
    GliderModelParams,
    GliderKinematicModelControl,
    GliderKinematicModelDisturbance,
    GliderOpenLoopKinematicModel,
)
from thermal_model.thermal_model import ThermalModel, ThermalModelParams
from thermal_estimator.thermal_estimator import ThermalEstimator
from utils.location import WorldFrameCoordinate
from controller.guidance_state_machine import GuidanceStateMachine, GuidanceState
from simulator.utils.airplane_glyph import draw_airplane
from simulator.visualization import SingleThermalSimVisualizer


@dataclass
class SingleThermalSimParams:
    glider_model_params: GliderModelParams
    thermal_model_params: ThermalModelParams
    dt: float = 0.1  # Default matches SIM_DT
    manual_mode: bool = False
    initial_altitude: float = 300.0


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


import dataclasses


class SingleThermalGliderSimulator:
    def __init__(self, sim_params: SingleThermalSimParams) -> None:
        self.sim_params = sim_params
        self.params = sim_params.glider_model_params
        self.dt = sim_params.dt

        self.glider = GliderOpenLoopKinematicModel(self.params)
        self.control = GliderKinematicModelControl(
            roll_angle=0.0, airspeed=self.params.V_star
        )
        self.disturbance = GliderKinematicModelDisturbance(
            thermal_uplift=0.0, wind_x=0.0, wind_y=0.0
        )
        self.ctrl_state = ControlState(roll_rad=0.0, airspeed_ms=self.params.V_star)

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

        scope_labels = list(self.scope_data.keys())
        self.visualizer = SingleThermalSimVisualizer(
            scope_labels=scope_labels,
            airplane_scale=AIRPLANE_SCALE,
            plot_span_xy=PLOT_SPAN_XY,
            plot_span_z=PLOT_SPAN_Z,
            plot_estimated_thermal_params=PLOT_ESTIMATED_THERMAL_PARAMS,
            scope_window_sec=SCOPE_WINDOW_SEC,
        )
        # Connect key event to the visualizer's figure
        self.visualizer.fig.canvas.mpl_connect("key_press_event", self._on_key)

        # --- Estimator ---
        self.thermal_estimator = ThermalEstimator(num_samples_to_buffer=50)
        self._step_count = 0
        self._time = 0.0
        self.thermal = ThermalModel(params=sim_params.thermal_model_params)

        # --- Guidance State Machine ---
        self.guidance_sm = GuidanceStateMachine(
            thermal_confidence_probe_threshold=0.3,
            thermal_confidence_circle_threshold=0.5,
            glider_model_params=self.params,
        )
        self.manual_mode = sim_params.manual_mode

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
        self.visualizer.draw(
            xs=self.xs,
            ys=self.ys,
            hs=self.hs,
            glider=self.glider,
            scope_data=self.scope_data,
            times=self.times,
            _time=self._time,
            _step_count=self._step_count,
            draw_airplane_func=draw_airplane,
            thermal=self.thermal,
            thermal_estimator=self.thermal_estimator,
        )

    def run(self) -> None:
        try:
            while True:
                self.step()
                # Always update 3D plot for smoothness
                self.draw(update_oscopes=(self._step_count % 10 == 0))
        except KeyboardInterrupt:
            pass


def run_single_thermal_sim(sim_params: SingleThermalSimParams) -> None:
    sim = SingleThermalGliderSimulator(sim_params)
    sim.run()


if __name__ == "__main__":
    # Load kinematic model parameters from JSON
    with open(
        os.path.join(os.path.dirname(__file__), "../ask21_kinematic_model.json"), "r"
    ) as f:
        model_params = json.load(f)

    glider_params = GliderModelParams(
        V_star=model_params["V_star"],
        V_stall=model_params["V_stall"],
        s_min=model_params["s_min"],
        k_v=model_params["k_v"],
        alpha_n=model_params["alpha_n"],
        initial_altitude=300.0,  # m
        roll_tau=0.2,  # s
        vel_tau=0.2,  # s
    )

    parser = argparse.ArgumentParser(description="Single Thermal Glider Simulator")
    parser.add_argument("--manual", action="store_true", help="Enable manual WASD mode")
    # Optionally, add more CLI args for param sweeps here
    args = parser.parse_args()

    thermal_params = ThermalModelParams()
    sim_params = SingleThermalSimParams(
        glider_model_params=glider_params,
        thermal_model_params=thermal_params,
        dt=SIM_DT,
        manual_mode=args.manual,
        initial_altitude=300.0,
    )
    run_single_thermal_sim(sim_params)
