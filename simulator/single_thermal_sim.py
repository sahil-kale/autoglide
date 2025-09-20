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
from controller.guidance_state_machine import GuidanceStateMachine
from simulator.utils.airplane_glyph import draw_airplane
from simulator.visualization import SingleThermalSimVisualizer
from vehicle_state_estimator.vehicle_state_estimator import (
    VehicleState,
    VehicleStateEstimatorPassthrough,
)
import dataclasses
from utils.vector import Vector2D
from simulator.constants import DefaultVisualizerParams
import json
import datetime
from simulator.constants import (
    SIM_DT,
    ROLL_STEP_RAD,
    VEL_STEP_MS,
    ROLL_LIMIT_RAD,
    V_MIN,
    V_MAX,
)
from simulator.logged_state import LoggedState


@dataclass
class SingleThermalSimParams:
    glider_model_params: GliderModelParams
    thermal_model_params: ThermalModelParams
    dt: float = 0.1  # Default matches SIM_DT
    manual_mode: bool = False
    initial_altitude: float = 300.0
    headless: bool = False
    sim_runtime: float = 60.0  # seconds, only used if headless
    log_save_path: str = "output"
    video_save_path: str = None  # only used if headless
    sim_title: str = "default_sim"


# --- Matplotlib keymap overrides (disable default bindings that conflict with controls) ---
mpl.rcParams["keymap.save"] = []  # 's'
mpl.rcParams["keymap.fullscreen"] = []  # 'f'
mpl.rcParams["keymap.pan"] = []  # 'p'
mpl.rcParams["keymap.zoom"] = []  # 'o', etc.


@dataclass
class ControlState:
    roll_rad: float
    airspeed_ms: float


class SingleThermalGliderSimulator:
    def __init__(self, sim_params: SingleThermalSimParams) -> None:
        self.sim_params = sim_params
        # --- Logging setup ---
        self.log_dir = os.path.join(
            self.sim_params.log_save_path,
            self.sim_params.sim_title,
        )
        os.makedirs(self.log_dir, exist_ok=True)
        self.log_path = os.path.join(self.log_dir, "sim_log.jsonl")
        self.log_file = open(self.log_path, "w")
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
        initial_state = VehicleState(
            position=WorldFrameCoordinate(self.glider.x, self.glider.y),
            airspeed=self.glider.V,
            velocity_ground=Vector2D(0.0, 0.0),
            heading=self.glider.psi,
            time=0.0,
        )
        self.vehicle_state_estimator = VehicleStateEstimatorPassthrough(initial_state)

        vis_params, self.scope_data = DefaultVisualizerParams.make(
            self.glider.h,
            self.glider.V,
            self.glider.phi,
            headless=sim_params.headless,
            video_save_path=sim_params.video_save_path,
        )
        self.visualizer = SingleThermalSimVisualizer(vis_params)
        if not sim_params.headless:
            self.visualizer.fig.canvas.mpl_connect("key_press_event", self._on_key)

        # --- Estimator ---
        self.thermal_estimator = ThermalEstimator(
            variometer_noise_std=0.7, num_samples_to_buffer=50
        )
        self._step_count = 0
        self._time = 0.0
        self.thermal = ThermalModel(params=sim_params.thermal_model_params)

        # --- Guidance State Machine ---
        self.guidance_sm = GuidanceStateMachine(
            thermal_confidence_circle_threshold=0.5,
            avg_thermal_strength_threshold_cruise_to_probe=0.5,
            circling_confidence_abort_threshold=0.2,
            min_probe_time_s=15.0,
            min_thermal_strength_to_circle=2.5,
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
        if self.thermal is not None:
            uplift = self.thermal.get_thermal_uplift(
                self.glider.x, self.glider.y, self.glider.h
            )
        else:
            uplift = 0.0
        self.disturbance.w = uplift
        location = WorldFrameCoordinate(self.glider.x, self.glider.y)
        self.thermal_estimator.step(uplift, location)
        thermal_estimate = self.thermal_estimator.get_estimate()
        self.vehicle_state_estimator.update(
            glider_position=location,
            true_airspeed=self.glider.V,
            true_heading=self.glider.psi,
            dt=self.dt,
        )
        vehicle_state = self.vehicle_state_estimator.get_state()
        origin_wp = WorldFrameCoordinate(0.0, 0.0)
        target_wp = WorldFrameCoordinate(1000.0, 0.0)
        control = self.guidance_sm.step(
            vehicle_state, thermal_estimate, origin_wp, target_wp
        )
        self.control.phi = control.phi
        self.control.V = control.V
        guidance_state_str = str(self.guidance_sm.get_state())
        if self.manual_mode:
            self.control.phi = self.ctrl_state.roll_rad
            self.control.V = self.ctrl_state.airspeed_ms
        self.glider.step(self.dt, self.control, self.disturbance)
        self.xs.append(self.glider.x)
        self.ys.append(self.glider.y)
        self.hs.append(self.glider.h)
        self._time += self.dt
        self._step_count += 1
        self.times.append(self._time)
        self.scope_data["Altitude (m)"].append(self.glider.h)
        self.scope_data["Airspeed (m/s)"].append(self.glider.V)
        self.scope_data["Roll (deg)"].append(np.rad2deg(self.glider.phi))
        self.scope_data["Uplift Speed (m/s)"].append(uplift)
        self.scope_data["Estimator Confidence"].append(thermal_estimate.confidence)
        self.scope_data["Guidance State"].append(guidance_state_str)
        log_entry = self.loggedstate_from_self()
        self.log_file.write(log_entry.to_json() + "\n")
        self.log_file.flush()
        self._last_loggedstate = log_entry

    def loggedstate_from_self(self) -> "LoggedState":
        from simulator.logged_state import LoggedState

        thermal_estimate = self.thermal_estimator.get_estimate()
        assert self.thermal is not None, "Thermal model is not defined in selfulator."
        actual_x, actual_y = self.thermal.core_center_at_height(self.glider.h)
        actual_r = getattr(self.thermal, "r_th", 0.0)
        return LoggedState(
            time=self._time,
            glider_x=self.glider.x,
            glider_y=self.glider.y,
            glider_h=self.glider.h,
            glider_V=self.glider.V,
            glider_phi=self.glider.phi,
            glider_phi_deg=np.rad2deg(self.glider.phi),
            glider_psi=self.glider.psi,
            control_phi=self.control.phi,
            control_V=self.control.V,
            disturbance_w=self.disturbance.w,
            estimator_confidence=self.scope_data["Estimator Confidence"][-1],
            guidance_state=self.scope_data["Guidance State"][-1],
            est_thermal_x=thermal_estimate.est_core.x,
            est_thermal_y=thermal_estimate.est_core.y,
            est_thermal_strength=thermal_estimate.W0,
            est_thermal_radius=thermal_estimate.Rth,
            actual_thermal_x=actual_x,
            actual_thermal_y=actual_y,
            actual_thermal_radius=actual_r,
        )

    # --- Drawing ---

    def draw(self) -> None:
        self.visualizer.draw(self._last_loggedstate)

    def run(self) -> None:
        try:
            if self.sim_params.headless:
                max_steps = int(self.sim_params.sim_runtime / self.dt)
                for _ in range(max_steps):
                    self.step()
                    self.draw()
                # Finalize video if needed
                if self.sim_params.video_save_path:
                    self.visualizer.finalize_video()
            else:
                while True:
                    self.step()
                    self.draw()
        finally:
            self.log_file.close()


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
        roll_tau=0.3,  # s
        vel_tau=0.3,  # s
        roll_rate_limit_rad_per_s=np.radians(60),  # rad/s
    )

    parser = argparse.ArgumentParser(description="Single Thermal Glider Simulator")
    parser.add_argument("--manual", action="store_true", help="Enable manual WASD mode")
    parser.add_argument(
        "--headless", action="store_true", help="Run in headless mode (no GUI)"
    )
    parser.add_argument(
        "--sim-runtime",
        type=float,
        default=30.0,
        help="Simulation runtime in seconds (headless only)",
    )
    parser.add_argument(
        "--video-save-path",
        type=str,
        default=None,
        help="Path to save video (headless only)",
    )
    # Optionally, add more CLI args for param sweeps here
    args = parser.parse_args()

    thermal_params = ThermalModelParams()
    thermal_params.x_c = 153.0  # m
    thermal_params.y_c = -25.8  # m
    sim_params = SingleThermalSimParams(
        glider_model_params=glider_params,
        thermal_model_params=thermal_params,
        dt=SIM_DT,
        manual_mode=args.manual,
        initial_altitude=300.0,
        headless=args.headless,
        sim_runtime=args.sim_runtime,
        video_save_path=args.video_save_path,
    )
    run_single_thermal_sim(sim_params)
