#!/usr/bin/env python3
"""
Single-window time-domain plotter for JSBSim sys-ID logs.

CSV schema (header must exist):
  t,phi,theta,p,q,r,beta,u_a,u_e,u_r,vt,h

Layout:
  ┌──────────────────────────── 3D view (trajectory + wireframe) ────────────────────────────┐
  └──────────────────────── Actuator + Euler angle (and optional rate) scopes ───────────────┘

Usage (live):
  python scripts/plot_sysid_timeseries_singlewin.py --csv output/sysid/sysid_log.csv

Save MP4 (requires ffmpeg):
  python scripts/plot_sysid_timeseries_singlewin.py --csv output/sysid/sysid_log.csv --save out.mp4
"""

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (registers 3D proj)
from matplotlib.gridspec import GridSpec

# ---------- helpers ----------

def load_log(csv_path: str):
    arr = np.genfromtxt(csv_path, delimiter=",", names=True)
    needed = ["t","phi","theta","p","q","r","beta","u_a","u_e","u_r","vt","h"]
    for k in needed:
        if k not in arr.dtype.names:
            raise ValueError(f"CSV missing column '{k}'. Found: {arr.dtype.names}")
    return arr

def integrate_yaw(t, r, psi0=0.0):
    psi = np.empty_like(r)
    psi[0] = psi0
    dt = np.diff(t)
    psi[1:] = psi[0] + np.cumsum(0.5*(r[:-1]+r[1:]) * dt)
    return np.unwrap(psi)

def integrate_position(t, vt, psi, h):
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    z = h.copy() 
    dt = np.diff(t)
    vx = vt * np.cos(psi)
    vy = vt * np.sin(psi)
    x[1:] = np.cumsum(0.5 * (vx[:-1] + vx[1:]) * dt)
    y[1:] = np.cumsum(0.5 * (vy[:-1] + vy[1:]) * dt)
    return x, y, z

def rotation_matrix(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
    return Rz @ Ry @ Rx

def glider_wireframe(scale=1.0):
    fus = np.array([[0.0,  1.0*scale],
                    [0.0,  0.0      ],
                    [0.0,  0.0      ]])
    wing = np.array([[-0.1*scale, -0.1*scale],
                     [-0.6*scale,  0.6*scale],
                     [0.0,         0.0      ]])
    tail = np.array([[-0.1*scale, -0.25*scale],
                     [0.0,         0.0       ],
                     [0.0,         0.0       ]])
    return [fus, wing, tail]

def transform_wireframe(segments, R, pos):
    out = []
    for seg in segments:
        pts = R @ seg + pos.reshape(3,1)
        out.append(pts)
    return out

# ---------- combined window ----------

def animate_single_window(
    t, phi, theta, psi, p, q, r, ua, ue, ur, x, y, z,
    show_rates=False, window_s=10.0, speed=1.0, save_path=None
):
    dt = float(t[1]-t[0])
    step = max(1, int(1/speed))
    N = len(t)
    win_n = max(2, int(round(window_s/dt)))

    # Figure layout: 3D on top (row 0), scopes below (rows 1..)
    rows = 2 + (1 if show_rates else 0)
    fig = plt.figure(figsize=(12, 8))
    gs = GridSpec(rows+1, 1, height_ratios=[2] + [1]*rows, hspace=0.3)

    # 3D axis
    ax3d = fig.add_subplot(gs[0, 0], projection='3d')

    # ---- Compute a constant box half-span R so axes stay equal and constant size ----
    # Use overall extents to pick a reasonable fixed radius, add a margin.
    xrange = np.max(x) - np.min(x)
    yrange = np.max(y) - np.min(y)
    zrange = np.max(z) - np.min(z)
    base_span = max(xrange, yrange, zrange)
    R = 10
    # Make axes scale equal
    try:
        ax3d.set_box_aspect((1, 1, 1))
    except Exception:
        pass  # older Matplotlib versions

    ax3d.set_xlabel("x [m]"); ax3d.set_ylabel("y [m]"); ax3d.set_zlabel("z [m]")
    ax3d.set_title("3D Trajectory + Wireframe (centered & equal-scale)")
    traj3d, = ax3d.plot([], [], [], lw=1)
    wf = glider_wireframe(scale=max(1.0, 0.1 * R))  # scale wireframe relative to box size
    wf_lines = [ax3d.plot([], [], [], lw=2)[0] for _ in wf]
    ax3d.view_init(elev=20, azim=-60)

    # Scopes
    ax_act = fig.add_subplot(gs[1, 0])
    l_ua, = ax_act.plot([], [], label="u_a (aileron)")
    l_ue, = ax_act.plot([], [], label="u_e (elevator)")
    l_ur, = ax_act.plot([], [], label="u_r (rudder)")
    ax_act.set_ylabel("Actuator (norm)")
    ax_act.legend(loc="upper right"); ax_act.grid(True)

    ax_ang = fig.add_subplot(gs[2, 0], sharex=ax_act)
    l_phi,   = ax_ang.plot([], [], label="phi [rad]")
    l_theta, = ax_ang.plot([], [], label="theta [rad]")
    l_psi,   = ax_ang.plot([], [], label="psi [rad]")
    ax_ang.set_ylabel("Angles [rad]")
    ax_ang.legend(loc="upper right"); ax_ang.grid(True)

    if show_rates:
        ax_rate = fig.add_subplot(gs[3, 0], sharex=ax_act)
        l_p, = ax_rate.plot([], [], label="p [rad/s]")
        l_q, = ax_rate.plot([], [], label="q [rad/s]")
        l_r, = ax_rate.plot([], [], label="r [rad/s]")
        ax_rate.set_ylabel("Rates [rad/s]")
        ax_rate.legend(loc="upper right"); ax_rate.grid(True)
        ax_rate.set_xlabel("Time [s]")
    else:
        ax_ang.set_xlabel("Time [s]")

    def init():
        # Initial centers/limits for 3D (center at first sample)
        cx, cy, cz = x[0], y[0], z[0]
        ax3d.set_xlim(cx - R, cx + R)
        ax3d.set_ylim(cy - R, cy + R)
        ax3d.set_zlim(cz - R, cz + R)

        # Limits for initial window on scopes
        ax_act.set_xlim(t[0], t[min(win_n, N-1)])
        ax_ang.set_xlim(t[0], t[min(win_n, N-1)])
        if show_rates:
            ax_rate.set_xlim(t[0], t[min(win_n, N-1)])
        return []

    def update(k):
        i = min(k*step, N-1)
        i0 = max(0, i - win_n)
        tx = t[i0:i+1]

        # --- 3D (centered around current pose with constant span R) ---
        traj3d.set_data(x[:i+1], y[:i+1])
        traj3d.set_3d_properties(z[:i+1])

        # Center axes on current position, keep equal and constant size
        cx, cy, cz = x[i], y[i], z[i]
        ax3d.set_xlim(cx - R, cx + R)
        ax3d.set_ylim(cy - R, cy + R)
        ax3d.set_zlim(cz - R, cz + R)

        Rb = rotation_matrix(phi[i], theta[i], psi[i])
        pos = np.array([cx, cy, cz])
        segs = transform_wireframe(wf, Rb, pos)
        for ln, seg in zip(wf_lines, segs):
            ln.set_data(seg[0, :], seg[1, :])
            ln.set_3d_properties(seg[2, :])

        # --- Scopes ---
        l_ua.set_data(tx, ua[i0:i+1])
        l_ue.set_data(tx, ue[i0:i+1])
        l_ur.set_data(tx, ur[i0:i+1])
        ax_act.relim(); ax_act.autoscale_view()
        ax_act.set_xlim(tx[0], tx[-1])

        l_phi.set_data(tx, phi[i0:i+1])
        l_theta.set_data(tx, theta[i0:i+1])
        l_psi.set_data(tx, psi[i0:i+1])
        ax_ang.relim(); ax_ang.autoscale_view()
        ax_ang.set_xlim(tx[0], tx[-1])

        if show_rates:
            l_p.set_data(tx, p[i0:i+1])
            l_q.set_data(tx, q[i0:i+1])
            l_r.set_data(tx, r[i0:i+1])
            ax_rate.relim(); ax_rate.autoscale_view()
            ax_rate.set_xlim(tx[0], tx[-1])

        return []

    ani = FuncAnimation(
        fig, update, init_func=init,
        frames=(N//step), interval=1000*dt*step, blit=False
    )

    if save_path:
        try:
            writer = FFMpegWriter(fps=int(1.0/(dt*step)))
            ani.save(save_path, writer=writer)
            print(f"[single-window] Saved animation to {save_path}")
        except Exception as e:
            print(f"[single-window] Could not save MP4 ({e}). Showing live window instead.")
            plt.show()
    else:
        plt.show()


# ---------- CLI ----------

def main():
    ap = argparse.ArgumentParser(description="Single-window 3D + oscilloscope plotter for JSBSim sys-ID logs")
    ap.add_argument("--csv", required=True, help="Path to sysid_log.csv")
    ap.add_argument("--window_s", type=float, default=10.0, help="Oscilloscope window [s]")
    ap.add_argument("--speed", type=float, default=1.0, help="Animation speed multiplier (>1 is faster)")
    ap.add_argument("--show-rates", action="store_true", help="Include p/q/r oscilloscope subplot")
    ap.add_argument("--save", type=str, default=None, help="Optional: save as MP4 (requires ffmpeg)")
    args = ap.parse_args()

    log = load_log(args.csv)
    t     = np.asarray(log["t"])
    phi   = np.asarray(log["phi"])
    theta = np.asarray(log["theta"])
    p     = np.asarray(log["p"])
    q     = np.asarray(log["q"])
    r     = np.asarray(log["r"])
    ua    = np.asarray(log["u_a"])
    ue    = np.asarray(log["u_e"])
    ur    = np.asarray(log["u_r"])
    vt    = np.asarray(log["vt"])
    h     = np.asarray(log["h"])

    psi = integrate_yaw(t, r, psi0=0.0)
    x, y, z = integrate_position(t, vt, psi, h)

    animate_single_window(
        t, phi, theta, psi, p, q, r, ua, ue, ur, x, y, z,
        show_rates=args.show_rates,
        window_s=args.window_s,
        speed=args.speed,
        save_path=args.save
    )

if __name__ == "__main__":
    main()
