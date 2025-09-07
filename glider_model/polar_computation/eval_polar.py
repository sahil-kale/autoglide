import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import scipy
import click

KMH_TO_MS = 1000 / 3600  # Conversion factor from km/h to m/s

def load_polar(polar_file):
    with open(polar_file, 'r') as f:
        polar_data = json.load(f)

    speeds = np.array(polar_data['speed_kmh']) * KMH_TO_MS  # Convert speeds to m/s
    sink_rates = np.array(polar_data['sink_rate_ms'])  # Sink rates in m/s
    assert len(speeds) == len(sink_rates), "Speeds and sink rates must have the same length."
    return speeds, sink_rates

def find_minsink(speeds, sink_rates):
    min_sink_idx = np.argmin(sink_rates)
    return speeds[min_sink_idx], sink_rates[min_sink_idx]

def fit_coefficients(speeds, sink_rates, V_star, s_min):
    # model: s0(V) = s_min + k_v * (V - V_star)^2
    # find k_v using least squares
    A = (speeds - V_star) ** 2
    b = sink_rates - s_min
    k_v, _, _, _ = np.linalg.lstsq(A[:, np.newaxis], b, rcond=None)
    return k_v[0]

def get_modelled_sink_rate_zero_bank(speed, V_star, s_min, k_v):
    return s_min + k_v * (speed - V_star) ** 2

def plot_polar(speeds, sink_rates, V_star, s_min, k_v):
    plt.figure(figsize=(10, 6))
    plt.plot(speeds, sink_rates, 'o', label='Measured Sink Rates')
    
    speed_range = np.linspace(min(speeds), max(speeds), 100)
    modelled_sink_rates = get_modelled_sink_rate_zero_bank(speed_range, V_star, s_min, k_v)
    plt.plot(speed_range, modelled_sink_rates, '-', label='Fitted Model', color='orange')
    
    plt.axvline(V_star, color='green', linestyle='--', label='Best Glide Speed (V*)')
    plt.axhline(s_min, color='red', linestyle='--', label='Minimum Sink Rate (s_min)')
    
    plt.xlabel('Speed (m/s)')
    plt.ylabel('Sink Rate (m/s)')
    plt.title('Glider Polar Curve, Actual vs Fitted Model')
    plt.legend()
    plt.grid()
    plt.gca().invert_yaxis()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--polar-file", type=str, required=True, help="Path to the polar JSON file.")
    parser.add_argument("--plot", action="store_true", help="Whether to plot the polar curve.")
    args = parser.parse_args()

    with open(args.polar_file, 'r') as f:
        polar_data = json.load(f)

    speeds, sink_rates = load_polar(args.polar_file)
    V_star, s_min = find_minsink(speeds, sink_rates)
    k_v = fit_coefficients(speeds, sink_rates, V_star, s_min)
    click.echo(f"Fitted coefficients:\nV* = {V_star:.2f} m/s\ns_min = {s_min:.2f} m/s\nk_v = {k_v:.4f} (m/s)/(m/s)^2")
    if args.plot:
        plot_polar(speeds, sink_rates, V_star, s_min, k_v)
