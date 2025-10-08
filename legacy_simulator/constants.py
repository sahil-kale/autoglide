import numpy as np


# --- Visualizer/Simulation Constants ---
PLOT_SPAN_XY = 100.0
PLOT_SPAN_Z = 50.0
AIRPLANE_SCALE = 6.0
SCOPE_WINDOW_SEC = 30.0
PLOT_ESTIMATED_THERMAL_PARAMS = True
SIM_DT = 0.1

ROLL_STEP_RAD = np.deg2rad(5.0)
VEL_STEP_MS = 1.0
ROLL_LIMIT_RAD = np.deg2rad(60.0)
V_MIN = 20.0
V_MAX = 50.0
DRAW_EVERY_STEPS = 2

import numpy as np


def default_scope_data(glider_h, glider_V, glider_phi):
    return {
        "Altitude (m)": [glider_h],
        "Airspeed (m/s)": [glider_V],
        "Roll (deg)": [np.rad2deg(glider_phi)],
        "Uplift Speed (m/s)": [0.0],
        "Estimator Confidence": [1.0],
        "Guidance State": ["Cruise"],
    }


class VisualizerParams:
    def __init__(
        self,
        scope_labels,
        airplane_scale=AIRPLANE_SCALE,
        plot_span_xy=PLOT_SPAN_XY,
        plot_span_z=PLOT_SPAN_Z,
        plot_estimated_thermal_params=PLOT_ESTIMATED_THERMAL_PARAMS,
        scope_window_sec=SCOPE_WINDOW_SEC,
        headless=False,
        video_save_path=None,
    ):
        self.scope_labels = scope_labels
        self.airplane_scale = airplane_scale
        self.plot_span_xy = plot_span_xy
        self.plot_span_z = plot_span_z
        self.plot_estimated_thermal_params = plot_estimated_thermal_params
        self.scope_window_sec = scope_window_sec
        self.headless = headless
        self.video_save_path = video_save_path


class DefaultVisualizerParams:
    @staticmethod
    def make(glider_h, glider_V, glider_phi, headless=False, video_save_path=None):
        scope_data = default_scope_data(glider_h, glider_V, glider_phi)
        scope_labels = list(scope_data.keys())
        return (
            VisualizerParams(
                scope_labels=scope_labels,
                airplane_scale=AIRPLANE_SCALE,
                plot_span_xy=PLOT_SPAN_XY,
                plot_span_z=PLOT_SPAN_Z,
                plot_estimated_thermal_params=PLOT_ESTIMATED_THERMAL_PARAMS,
                scope_window_sec=SCOPE_WINDOW_SEC,
                headless=headless,
                video_save_path=video_save_path,
            ),
            scope_data,
        )


class VisualizerParams:
    def __init__(
        self,
        scope_labels,
        airplane_scale=AIRPLANE_SCALE,
        plot_span_xy=PLOT_SPAN_XY,
        plot_span_z=PLOT_SPAN_Z,
        plot_estimated_thermal_params=PLOT_ESTIMATED_THERMAL_PARAMS,
        scope_window_sec=SCOPE_WINDOW_SEC,
        headless=False,
        video_save_path=None,
    ):
        self.scope_labels = scope_labels
        self.airplane_scale = airplane_scale
        self.plot_span_xy = plot_span_xy
        self.plot_span_z = plot_span_z
        self.plot_estimated_thermal_params = plot_estimated_thermal_params
        self.scope_window_sec = scope_window_sec
        self.headless = headless
        self.video_save_path = video_save_path
