import os
import sys
from copy import deepcopy

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator.single_thermal_sim import (
    SingleThermalSimParams,
    SingleThermalGliderSimulator,
)
import numpy as np
import json
from glider_model.model import GliderModelParams
from thermal_model.thermal_model import ThermalModelParams
import datetime

NUM_SIMS_THERMAL_CENTER_OFFSET = 50

ASK21_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), "../ask21_kinematic_model.json"
)
LOG_OUTPUT_DIR = "output/monte_carlo"


def run_thermal_center_offset_analysis(
    default_sim_params: SingleThermalSimParams, output_dir
):
    y_offset_mu = 0.0
    y_offset_sigma = 20.0
    x_offset_mu = 150.0
    x_offset_sigma = 10.0
    sim_params = deepcopy(default_sim_params)

    sim_params.thermal_model_params.y_c = 0.0
    for i in range(NUM_SIMS_THERMAL_CENTER_OFFSET):
        x_offset = np.random.normal(x_offset_mu, x_offset_sigma)
        y_offset = np.random.normal(y_offset_mu, y_offset_sigma)
        sim_params.thermal_model_params.x_c = x_offset
        sim_params.thermal_model_params.y_c = y_offset
        sim_params.sim_title = f"thermal_offset_x_{x_offset:.1f}_y_{y_offset:.1f}"
        sim_params.video_save_path = f"{output_dir}/{sim_params.sim_title}/video.mp4"
        print(
            f"Running simulation {i+1}/{NUM_SIMS_THERMAL_CENTER_OFFSET} with thermal x offset: {x_offset:.1f} m, y offset: {y_offset:.1f} m"
        )
        sim = SingleThermalGliderSimulator(sim_params)
        sim.run()


def run_monte_carlo_simulations():
    # seed for reproducibility
    np.random.seed(42)
    output_dir = f"{LOG_OUTPUT_DIR}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"

    os.makedirs(output_dir, exist_ok=True)

    with open(ASK21_MODEL_PATH, "r") as f:
        model_params = json.load(f)

    glider_params = GliderModelParams(
        V_star=model_params["V_star"],
        V_stall=model_params["V_stall"],
        s_min=model_params["s_min"],
        k_v=model_params["k_v"],
        alpha_n=model_params["alpha_n"],
        initial_altitude=300.0,  # m
        roll_tau=0.6,  # s
        vel_tau=0.5,  # s
    )

    default_thermal_params = ThermalModelParams(
        w_max=7.0,
        r_th=50.0,
        x_c=0.0,
        y_c=0.0,
        V_e=0.0,
        kx=0.0,
        ky=0.0,
        core_center_random_noise_std=2.0,
    )

    default_sim_params = SingleThermalSimParams(
        glider_model_params=glider_params,
        thermal_model_params=default_thermal_params,
        headless=True,
        sim_runtime=30.0,
        log_save_path=output_dir,
    )
    run_thermal_center_offset_analysis(default_sim_params, output_dir)


if __name__ == "__main__":
    run_monte_carlo_simulations()
