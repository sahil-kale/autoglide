import json
import click
import os
import argparse
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt
import math

STEADY_STATE_SAMPLES_FROM_END = 50


@dataclass
class ThermalOffsetResults:
    steady_state_estimated_core_distance_error_m: float = 0.0
    steady_state_estimated_radius_m: float = 0.0
    steady_state_estimated_w_max_m_per_s: float = 0.0
    steady_state_climb_rate_m_per_s: float = 0.0
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
    for i, entry in enumerate(data[-STEADY_STATE_SAMPLES_FROM_END:]):
        distance_error = (
            (entry["est_thermal_x"] - entry["actual_thermal_x"]) ** 2
            + (entry["est_thermal_y"] - entry["actual_thermal_y"]) ** 2
        ) ** 0.5
        summed_distance_error += distance_error
        summed_radius += entry["est_thermal_radius"]
        summed_w_max += entry["est_thermal_strength"]

        idx = len(data) - STEADY_STATE_SAMPLES_FROM_END + i
        climb_delta = data[idx]["glider_h"] - data[idx - 1]["glider_h"]
        dt = data[idx]["time"] - data[idx - 1]["time"]
        climb_rate = climb_delta / dt if dt > 0 else 0.0
        summed_climb_rate += climb_rate

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
    results.success = (
        data[-1]["guidance_state"] == "Circle"
        and results.steady_state_climb_rate_m_per_s > 2.0
    )

    if results.success:
        for i, entry in enumerate(data):
            if (entry["guidance_state"] == "Circle") and (
                results.time_to_switch_to_circling_s == 0.0
            ):
                results.time_to_switch_to_circling_s = entry["time"]

    return results


def collect_all_thermal_offset_results(files):
    all_results = []
    for file in files:
        try:
            all_results.append(analyze_thermal_center_offset_results(file))
        except Exception as e:
            click.secho(f"Error analyzing {file}: {e}", fg="red")
    return all_results


def _hist(ax, data, title, xlabel, bins="auto"):
    arr = np.asarray([d for d in data if d is not None])
    n = len(arr)
    if n == 0:
        ax.text(0.5, 0.5, "No data", ha="center", va="center", fontsize=10)
        ax.set_title(title)
        ax.set_xlabel(xlabel)
        ax.set_yticks([])
        ax.grid(True, alpha=0.25)
        return

    ax.hist(arr, bins=bins, alpha=0.7, edgecolor="black")
    mu = float(np.mean(arr))
    med = float(np.median(arr))
    ax.axvline(mu, linestyle="--", linewidth=1.5, label=f"mean = {mu:.3g}")
    ax.axvline(med, linestyle="-.", linewidth=1.5, label=f"median = {med:.3g}")

    ax.set_title(f"{title}  (n={n})")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("Count")
    ax.grid(True, alpha=0.25)
    ax.legend(fontsize=8, loc="upper right")


def _bar_success(ax, n_success, n_fail):
    total = n_success + n_fail
    ax.bar(
        ["Successful", "Unsuccessful"],
        [n_success, n_fail],
        edgecolor="black",
        alpha=0.8,
    )
    for i, v in enumerate([n_success, n_fail]):
        pct = (100.0 * v / total) if total > 0 else 0.0
        ax.text(i, v, f"{v} ({pct:.1f}%)", ha="center", va="bottom", fontsize=9)
    ax.set_title(f"Run Outcomes  (total={total})")
    ax.set_ylabel("Count")
    ax.grid(axis="y", alpha=0.25)


def plot_thermal_offset_results(all_results, save_path=None, show=True):
    """
    Plot summary histograms for thermal offset evaluation results,
    plus a success vs unsuccessful bar chart.

    Args:
        all_results: iterable of result objects with attributes:
            - success (bool)
            - steady_state_estimated_core_distance_error_m
            - steady_state_estimated_radius_m
            - steady_state_estimated_w_max_m_per_s
            - steady_state_climb_rate_m_per_s
            - time_to_switch_to_circling_s
        save_path: optional filepath to save the figure (e.g., 'results.png' or 'results.pdf')
        show: whether to plt.show() the figure
    """
    if not all_results:
        click.secho("No results to plot.", fg="red")
        return

    successful = [r for r in all_results if getattr(r, "success", False)]
    unsuccessful = [r for r in all_results if not getattr(r, "success", False)]
    if not successful:
        click.secho("No successful results to plot.", fg="yellow")
        # Still plot the outcomes bar so you can see zeros
        distance_errors = radii = w_maxs = climb_rates = times_to_circle = []
    else:
        distance_errors = [
            r.steady_state_estimated_core_distance_error_m for r in successful
        ]
        radii = [r.steady_state_estimated_radius_m for r in successful]
        w_maxs = [r.steady_state_estimated_w_max_m_per_s for r in successful]
        climb_rates = [r.steady_state_climb_rate_m_per_s for r in successful]
        times_to_circle = [
            r.time_to_switch_to_circling_s
            for r in successful
            if getattr(r, "time_to_switch_to_circling_s", 0)
            and r.time_to_switch_to_circling_s > 0
        ]

    plots = [
        (distance_errors, "Core Distance Error (m)", "Distance Error (m)"),
        (radii, "Estimated Radius (m)", "Radius (m)"),
        (w_maxs, "Estimated W_max (m/s)", "W_max (m/s)"),
        (climb_rates, "Steady-State Climb (m/s)", "Climb Rate (m/s)"),
        (times_to_circle, "Time to Circling (s)", "Time (s)"),
    ]

    # +1 panel for outcomes bar chart
    n = len(plots) + 1
    cols = min(3, n)
    rows = math.ceil(n / cols)

    fig, axes = plt.subplots(rows, cols, figsize=(4.8 * cols, 3.8 * rows))
    if isinstance(axes, np.ndarray):
        axes = axes.ravel()
    else:
        axes = [axes]

    # Histograms
    for i, (data, title, xlabel) in enumerate(plots):
        _hist(axes[i], data, title, xlabel, bins="auto")

    # Success vs unsuccessful bar chart in the next slot
    _bar_success(axes[len(plots)], len(successful), len(unsuccessful))

    # Hide any extra axes
    for j in range(n, len(axes)):
        axes[j].axis("off")

    fig.suptitle("Thermal Offset Monte Carlo Analysis", fontsize=14, y=0.98)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    if save_path:
        fig.savefig(save_path, dpi=200, bbox_inches="tight")
        click.secho(f"Saved plots to: {save_path}", fg="green")

    if show:
        plt.show()
    else:
        plt.close(fig)


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
