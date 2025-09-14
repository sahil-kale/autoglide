import numpy as np
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from utils.location import WorldFrameCoordinate
from utils.vector import Vector2D

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse


class L1GuidanceLaw:
    def __init__(self):
        self.g = 9.81  # Gravity (m/s^2)

    def compute_lateral_acceleration_for_waypoint(
        self,
        ground_speed_vector: Vector2D,
        current_location: WorldFrameCoordinate,
        lookahead_point: WorldFrameCoordinate,
    ):
        l1_vector = lookahead_point - current_location
        l1_distance = l1_vector.norm()
        eta = ground_speed_vector.signed_angle_between(l1_vector)
        ground_speed = ground_speed_vector.norm()
        a_s_cmd = 2 * ground_speed**2 / self.l1_distance * np.sin(eta)
        return a_s_cmd

    def compute_bank_angle_for_lateral_acceleration(self, a_s_cmd):
        return np.arctan2(a_s_cmd, self.g)


def simulate_l1_guidance(mode="line", save_path="l1_guidance_sim.mp4"):
    dt = 0.1
    T = 30.0
    steps = int(T / dt)
    l1 = L1GuidanceLaw()
    l1.l1_distance = 30.0
    V = 25.0
    if mode == "line":
        x, y = -100.0, 30.0
        psi = np.deg2rad(-10)
        def get_lookahead(x, y):
            return WorldFrameCoordinate(x + l1.l1_distance, 0.0)
    elif mode == "circle":
        R = 50.0
        x, y = R + 30.0, 0.0
        psi = np.deg2rad(45)
        def get_lookahead(x, y):
            th = np.arctan2(y, x)
            th_target = th + l1.l1_distance / R
            return WorldFrameCoordinate(R * np.cos(th_target), R * np.sin(th_target))
    else:
        raise ValueError("mode must be 'line' or 'circle'")

    traj = []
    for _ in range(steps):
        loc = WorldFrameCoordinate(x, y)
        v_g = Vector2D(V * np.cos(psi), V * np.sin(psi))
        look = get_lookahead(x, y)
        a_cmd = l1.compute_lateral_acceleration_for_waypoint(v_g, loc, look)
        phi = l1.compute_bank_angle_for_lateral_acceleration(a_cmd)
        psi += l1.g / V * np.tan(phi) * dt
        x += V * np.cos(psi) * dt
        y += V * np.sin(psi) * dt
        traj.append((x, y))

    traj = np.array(traj)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect('equal')
    ax.grid(True)
    if mode == "line":
        ax.plot([-100, 100], [0, 0], 'k--', label="target line")
    else:
        ax.add_patch(plt.Circle((0, 0), R, color='k', fill=False, linestyle='--', label="target circle"))
    path, = ax.plot([], [], 'b-', lw=2)
    glider, = ax.plot([], [], 'ro', markersize=8)
    ax.legend()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)

    def animate(i):
        path.set_data(traj[:i, 0], traj[:i, 1])
        glider.set_data([traj[i, 0]], [traj[i, 1]])
        return path, glider

    ani = animation.FuncAnimation(fig, animate, frames=steps, interval=30, blit=True, repeat=False)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="L1 Guidance Law 2D Simulation")
    parser.add_argument('--mode', choices=['line', 'circle'], default='line', help='Tracking mode: line or circle')
    parser.add_argument('--output', type=str, default='l1_guidance_sim.mp4', help='Output video file path')
    args = parser.parse_args()
    simulate_l1_guidance(mode=args.mode, save_path=args.output)
