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
    # Simulation parameters
    dt = 0.1
    total_time = 30.0
    steps = int(total_time / dt)
    l1 = L1GuidanceLaw()
    l1.l1_distance = 30.0
    V = 25.0  # m/s
    positions = []
    headings = []
    bank_angles = []

    # Initial state: start offset from the path
    if mode == "line":
        x, y = -100.0, 30.0  # Start 30m above the line y=0
        psi = np.deg2rad(-10)  # Slight heading offset
        def get_lookahead_point(x, y):
            # Project forward along y=0
            return WorldFrameCoordinate(x + l1.l1_distance, 0.0)
    elif mode == "circle":
        R = 50.0
        x, y = R + 30.0, 0.0  # Start 30m outside the circle
        psi = np.deg2rad(45)  # Heading offset
        def get_lookahead_point(x, y):
            theta = np.arctan2(y, x)
            theta_target = theta + l1.l1_distance / R
            return WorldFrameCoordinate(R * np.cos(theta_target), R * np.sin(theta_target))
    else:
        raise ValueError("Unknown mode: choose 'line' or 'circle'")

    for step in range(steps):
        current_loc = WorldFrameCoordinate(x, y)
        ground_vec = Vector2D(V * np.cos(psi), V * np.sin(psi))
        lookahead = get_lookahead_point(x, y)
        a_s_cmd = l1.compute_lateral_acceleration_for_waypoint(ground_vec, current_loc, lookahead)
        phi_cmd = l1.compute_bank_angle_for_lateral_acceleration(a_s_cmd)
        # Heading rate: psi_dot = g/V * tan(phi)
        psi_dot = l1.g / V * np.tan(phi_cmd)
        psi += psi_dot * dt
        x += V * np.cos(psi) * dt
        y += V * np.sin(psi) * dt
        positions.append((x, y))
        headings.append(psi)
        bank_angles.append(phi_cmd)

    positions = np.array(positions)

    # Animation
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_aspect('equal')
    ax.grid(True)
    if mode == "line":
        ax.plot([-100, 100], [0, 0], 'k--', label="Target Line")
    else:
        circle = plt.Circle((0, 0), R, color='k', fill=False, linestyle='--', label="Target Circle")
        ax.add_patch(circle)
    path_line, = ax.plot([], [], 'b-', lw=2, label="Glider Path")
    glider_dot, = ax.plot([], [], 'ro', markersize=8, label="Glider")
    ax.legend()
    ax.set_xlim(-120, 120)
    ax.set_ylim(-120, 120)

    def init():
        path_line.set_data([], [])
        glider_dot.set_data([], [])
        return path_line, glider_dot

    def animate(i):
        path_line.set_data(positions[:i, 0], positions[:i, 1])
        # glider_dot expects sequences, not scalars
        glider_dot.set_data([positions[i, 0]], [positions[i, 1]])
        return path_line, glider_dot

    ani = animation.FuncAnimation(fig, animate, frames=steps, init_func=init,
                                  interval=30, blit=True, repeat=False)
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="L1 Guidance Law 2D Simulation")
    parser.add_argument('--mode', choices=['line', 'circle'], default='line', help='Tracking mode: line or circle')
    parser.add_argument('--output', type=str, default='l1_guidance_sim.mp4', help='Output video file path')
    args = parser.parse_args()
    simulate_l1_guidance(mode=args.mode, save_path=args.output)
