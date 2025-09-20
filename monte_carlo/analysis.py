import json
import click
import os
import argparse
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt

STEADY_STATE_SAMPLES_FROM_END = 50


@dataclass
class ThermalOffsetResults:
    steady_state_estimated_core_distance_error_m: float = 0.0
    steady_state_estimated_radius_m: float = 0.0
    steady_state_estimated_w_max_m_per_s: float = 0.0
    steady_state_climb_rate_m_per_s: float = 0.0
    time_to_reach_steady_state_s: float = 0.0
    time_to_switch_to_circling_s: float = 0.0
    success: bool = False


def load_log_data(file):
    data = []
    with open(file, "r") as f:
        for line in f:
            try:
                entry = json.loads(line)
                data.append(entry)
            except json.JSONDecodeError as e:
                click.secho(f"Error decoding JSON line: {line}. Error: {e}", fg="red")
    return data


def analyze_thermal_center_offset_results(file):
    # {"time": 60.00000000000058, "glider_x": 146.93632777923406, "glider_y": -12.488863226489064, "glider_h": 475.32605748875625, "glider_V": 18.500000000000007, "glider_phi": 1.114081176237776, "glider_phi_deg": 63.832149433395024, "glider_psi": 2.270794855253962, "control_phi": 1.117182146360778, "control_V": 18.5, "disturbance_w": 5.89991063322526, "estimator_confidence": 0.9388846908290144, "guidance_state": "Circle", "est_thermal_x": 132.9844925705187, "est_thermal_y": -22.876546719372477, "est_thermal_strength": 7.139632041052124, "est_thermal_radius": 44.23410897645216, "actual_thermal_x": 131.31979570389922, "actual_thermal_y": -20.248502745939053, "actual_thermal_radius": 50.0}
    data = load_log_data(file)

    if not data:
        click.secho(f"No valid data found in {file}.", fg="red")
        return

    # Analyze the data
    results = ThermalOffsetResults()
    summed_distance_error = 0.0
    summed_radius = 0.0
    summed_w_max = 0.0
    summed_climb_rate = 0.0
    for entry in data[-STEADY_STATE_SAMPLES_FROM_END:]:
        distance_error = (
            (entry["est_thermal_x"] - entry["actual_thermal_x"]) ** 2
            + (entry["est_thermal_y"] - entry["actual_thermal_y"]) ** 2
        ) ** 0.5
        summed_distance_error += distance_error
        summed_radius += entry["est_thermal_radius"]
        summed_w_max += entry["est_thermal_strength"]
        summed_climb_rate += entry["disturbance_w"]

    results.steady_state_estimated_core_distance_error_m = (
        summed_distance_error / STEADY_STATE_SAMPLES_FROM_END
    )
    results.steady_state_estimated_radius_m = (
        summed_radius / STEADY_STATE_SAMPLES_FROM_END
    )
    results.steady_state_estimated_w_max_m_per_s = (
        summed_w_max / STEADY_STATE_SAMPLES_FROM_END
    )
    results.steady_state_climb_rate_m_per_s = (
        summed_climb_rate / STEADY_STATE_SAMPLES_FROM_END
    )
    results.success = data[-1]["guidance_state"] == "Circle"

    if results.success:
        for i, entry in enumerate(data):
            if entry["guidance_state"] == "Circle":
                results.time_to_switch_to_circling_s = entry["time"]
                break

    return results

def collect_all_thermal_offset_results(files):
    all_results = []
    for file in files:
        try:
            analyze_thermal_center_offset_results(file)
        except Exception as e:
            click.secho(f"Error analyzing {file}: {e}", fg="red")
    return all_results

def plot_thermal_offset_results(all_results):
    if not all_results:
        click.secho("No results to plot.", fg="red")
        return

    # Extract data for plotting
    distance_errors = [res.steady_state_estimated_core_distance_error_m for res in all_results]
    radii = [res.steady_state_estimated_radius_m for res in all_results]
    w_maxs = [res.steady_state_estimated_w_max_m_per_s for res in all_results]
    climb_rates = [res.steady_state_climb_rate_m_per_s for res in all_results]
    times_to_steady = [res.time_to_reach_steady_state_s for res in all_results if res.time_to_reach_steady_state_s > 0]
    times_to_circle = [res.time_to_switch_to_circling_s for res in all_results if res.time_to_switch_to_circling_s > 0]

    # Create subplots
    fig, axs = plt.subplots(3, 2, figsize=(12, 10))
    axs[0, 0].hist(distance_errors, bins=20, color='blue', alpha=0.7)
    axs[0, 0].set_title('Steady State Estimated Core Distance Error (m)')
    axs[0, 0].set_xlabel('Distance Error (m)')
    axs[0, 0].set_ylabel('Frequency')

    axs[0, 1].hist(radii, bins=20, color='green', alpha=0.7)
    axs[0, 1].set_title('Steady State Estimated Radius (m)')
    axs[0, 1].set_xlabel('Radius (m)')
    axs[0, 1].set_ylabel('Frequency')

    axs[1, 0].hist(w_maxs, bins=20, color='red', alpha=0.7)
    axs[1, 0].set_title('Steady State Estimated W_max (m/s)')
    axs[1, 0].set_xlabel('W_max (m/s)')
    axs[1, 0].set_ylabel('Frequency')

    axs[1, 1].hist(climb_rates, bins=20, color='purple', alpha=0.7)
    axs[1, 1].set_title('Steady State Climb Rate (m/s)')
    axs[1, 1].set_xlabel('Climb Rate (m/s)')
    axs[1, 1].set_ylabel('Frequency')

    plt.tight_layout()
    plt.show()

def get_thermal_offset_files(output_dir):
    thermal_offset_files = []
    THERMAL_OFFSET_FILE_IDENTIFIER = "thermal_offset"
    # find all of the directories in output_dir that contain THERMAL_OFFSET_FILE_IDENTIFIER
    for root, dirs, files in os.walk(output_dir):
        for dir_name in dirs:
            if THERMAL_OFFSET_FILE_IDENTIFIER in dir_name:
                # append any json files in this directory to the list
                dir_path = os.path.join(root, dir_name)
                for file_name in os.listdir(dir_path):
                    if file_name.endswith(".jsonl"):
                        thermal_offset_files.append(os.path.join(dir_path, file_name))
    return thermal_offset_files


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Analyze Monte Carlo simulation results for thermal center offset."
    )
    parser.add_argument(
        "output_dir",
        type=str,
        help="Directory containing simulation output.",
    )
    args = parser.parse_args()

    output_dir = args.output_dir
    if not os.path.exists(output_dir):
        raise FileNotFoundError(f"Output directory {output_dir} does not exist.")

    thermal_offset_files = get_thermal_offset_files(output_dir)
    if not thermal_offset_files:
        raise FileNotFoundError(
            f"No thermal offset result files found in {output_dir}."
        )

    click.secho(
        f"Found {len(thermal_offset_files)} thermal offset result files.", fg="green"
    )

    all_results = collect_all_thermal_offset_results(thermal_offset_files)
    click.secho(f"Collected results from {len(all_results)} simulations.", fg="green")
    plot_thermal_offset_results(all_results)
