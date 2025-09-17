import os
import sys
import json
import argparse
import numpy as np
import matplotlib as mpl

# Ensure package root is on path (one level up from this file)
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from simulator.visualization import SingleThermalSimVisualizer
from simulator.constants import DefaultVisualizerParams
from simulator.logged_state import LoggedState

# --- Matplotlib keymap overrides (disable default bindings that conflict with controls) ---
mpl.rcParams["keymap.save"] = []  # 's'
mpl.rcParams["keymap.fullscreen"] = []  # 'f'
mpl.rcParams["keymap.pan"] = []  # 'p'
mpl.rcParams["keymap.zoom"] = []  # 'o', etc.


def load_log(log_path):
    with open(log_path, "r") as f:
        for line in f:
            yield json.loads(line)


def main():
    parser = argparse.ArgumentParser(description="Playback a simulation log file.")
    parser.add_argument("log_path", type=str, help="Path to .jsonl log file")
    parser.add_argument(
        "--headless", action="store_true", help="Run in headless mode (no GUI)"
    )
    parser.add_argument(
        "--video-save-path",
        type=str,
        default=None,
        help="Path to save video (headless only)",
    )
    parser.add_argument(
        "--fps", type=float, default=10.0, help="Playback frames per second"
    )
    args = parser.parse_args()

    log_entries = list(load_log(args.log_path))
    if not log_entries:
        print("No log entries found.")
        return

    # Use the first log entry to get initial values for scope_data
    first = log_entries[0]
    glider_h = first.get("glider_h", 0.0)
    glider_V = first.get("glider_V", 0.0)
    glider_phi = first.get("glider_phi", 0.0)
    vis_params, _ = DefaultVisualizerParams.make(
        glider_h,
        glider_V,
        glider_phi,
        headless=args.headless,
        video_save_path=args.video_save_path,
    )
    visualizer = SingleThermalSimVisualizer(vis_params)

    def to_loggedstate(d):
        return LoggedState.from_json(d)

    if args.headless:
        for entry in log_entries:
            visualizer.draw(to_loggedstate(entry))
        if args.video_save_path:
            visualizer.finalize_video()
    else:
        import time

        for entry in log_entries:
            visualizer.draw(to_loggedstate(entry))
            mpl.pyplot.pause(1.0 / args.fps)
        mpl.pyplot.show()


if __name__ == "__main__":
    main()
