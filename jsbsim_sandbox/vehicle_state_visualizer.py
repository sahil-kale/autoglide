import math
from typing import List, Callable, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.animation import FuncAnimation

# Keep a global ref so Matplotlib doesn't GC the animation
_anim_ref = None


def _nice_limits(vmin: float, vmax: float, pad: float = 10.0) -> Tuple[float, float]:
    if not np.isfinite(vmin) or not np.isfinite(vmax):
        return -1.0, 1.0
    if abs(vmax - vmin) < 1e-6:
        c = 0.5 * (vmax + vmin)
        return c - pad, c + pad
    return vmin - pad, vmax + pad


def latlon_to_local_xy(
    lat_deg: np.ndarray, lon_deg: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Flat-earth local tangent plane in meters; origin = first sample."""
    if len(lat_deg) == 0:
        return np.array([]), np.array([])
    lat0 = math.radians(float(lat_deg[0]))
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(lat0)
    x = (lon_deg - lon_deg[0]) * m_per_deg_lon
    y = (lat_deg - lat_deg[0]) * m_per_deg_lat
    return x, y


def aircraft_wireframe(scale=5.0):
    """Body-frame wireframe: +X fwd, +Y right, +Z down."""
    fuselage = 6.0 * (scale / 5.0)
    half_wing = 8.0 * (scale / 5.0)
    tail = 2.5 * (scale / 5.0)
    triad = 3.0 * (scale / 5.0)

    pts, segs = [], []
    # fuselage
    pts += [[+fuselage / 2, 0, 0], [-fuselage / 2, 0, 0]]
    segs += [(0, 1)]
    idx = 2
    # wings
    pts += [[0, +half_wing, 0], [0, -half_wing, 0]]
    segs += [(idx, idx + 1)]
    idx += 2
    # tailplane
    pts += [[-fuselage / 2, +tail, 0], [-fuselage / 2, -tail, 0]]
    segs += [(idx, idx + 1)]
    idx += 2
    # fin (down)
    pts += [[-fuselage / 2, 0, 0], [-fuselage / 2, 0, +tail]]
    segs += [(idx, idx + 1)]
    idx += 2
    # axes triad
    pts += [[0, 0, 0], [triad, 0, 0]]
    segs += [(idx, idx + 1)]
    idx += 2
    pts += [[0, 0, 0], [0, triad, 0]]
    segs += [(idx, idx + 1)]
    idx += 2
    pts += [[0, 0, 0], [0, 0, triad]]
    segs += [(idx, idx + 1)]
    idx += 2
    return np.array(pts, float), segs


def R_from_euler_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    World-from-body rotation for ZYX (yaw-pitch-roll).
    roll = φ about X, pitch = θ about Y, yaw = ψ about Z.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    return Rz @ Ry @ Rx  # body -> world


def animate_sim(
    states: List,
    *,
    euler_from_state: Callable[
        [object], Tuple[float, float, float]
    ] = lambda s: s.attitude.get_euler(),
    yaw_left_positive: bool = True,
    flip_body_z_for_display: bool = True,
    interval_ms: int = 30,
    frame_step: int = 2,  # decimate frames for speed (2 = every other sample)
    wire_scale: float = 5.0,
):
    """
    Fast flight playback visualizer.

    - Uses s.attitude.get_euler() -> (roll, pitch, yaw) [rad].
    - If your convention has r>0 = yaw-left (common in your notes), set yaw_left_positive=True
      and we will flip yaw sign to match right-handed Z-up display yaw.
    - flip_body_z_for_display: if your body frame is +Z down (aero), keep True to flip for Z-up display.

    Returns the Matplotlib animation. Keep a reference: anim = animate_sim(...).
    """
    assert len(states) >= 2, "Need at least 2 samples to animate."
    # sort by time (defensive)
    states = sorted(states, key=lambda s: float(s.time_s))

    # Bulk arrays
    t = np.array([float(s.time_s) for s in states], dtype=float)
    alt = np.array([float(s.altitude_m) for s in states], dtype=float)
    tas = np.array([float(s.airspeed_mps) for s in states], dtype=float)
    p = np.array([float(s.p_radps) for s in states], dtype=float)
    q = np.array([float(s.q_radps) for s in states], dtype=float)
    r = np.array([float(s.r_radps) for s in states], dtype=float)
    lat = np.array([float(s.latitude_deg) for s in states], dtype=float)
    lon = np.array([float(s.longitude_deg) for s in states], dtype=float)

    # Positions (local tangent XY), Z up = altitude
    X, Y = latlon_to_local_xy(lat, lon)
    Z = alt

    # Euler angles (rad) from the provided function
    eulers = np.array([euler_from_state(s) for s in states], dtype=float)
    roll = eulers[:, 0]
    pitch = eulers[:, 1]
    yaw = eulers[:, 2]

    # If your convention is r>0 = yaw-left, flip to match right-handed Z-up display yaw
    if yaw_left_positive:
        yaw = -yaw

    roll_deg, pitch_deg, yaw_deg = np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    # Frame decimation for speed
    idx_all = np.arange(len(t))
    idx_anim = idx_all[:: max(1, frame_step)]
    t_anim = t[idx_anim]

    # Figure & layout
    fig = plt.figure(figsize=(12, 7))
    gs = gridspec.GridSpec(
        2,
        3,
        figure=fig,
        width_ratios=[1.4, 1.0, 1.0],
        height_ratios=[1.0, 1.0],
        wspace=0.35,
        hspace=0.35,
    )
    ax3d = fig.add_subplot(gs[:, 0], projection="3d")
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_tas = fig.add_subplot(gs[1, 1])
    ax_rates = fig.add_subplot(gs[0, 2])
    ax_euler = fig.add_subplot(gs[1, 2])
    fig.suptitle("Flight Playback Viewer (fast)", fontsize=14)

    # 3D track
    (track_full,) = ax3d.plot(X, Y, Z, linewidth=1, alpha=0.3)
    (track_live,) = ax3d.plot([], [], [], linewidth=2)

    # Wireframe in body frame
    body_pts_b, body_segs = aircraft_wireframe(scale=wire_scale)
    if flip_body_z_for_display:
        body_pts_disp = body_pts_b.copy()
        body_pts_disp[:, 2] *= -1.0  # convert body Z-down to display Z-up
    else:
        body_pts_disp = body_pts_b

    # Body segment artists
    body_lines = []
    for i0, i1 in body_segs:
        (line,) = ax3d.plot([], [], [], linewidth=2)
        body_lines.append((line, i0, i1))

    # Limits
    xlo, xhi = _nice_limits(float(np.nanmin(X)), float(np.nanmax(X)), pad=10.0)
    ylo, yhi = _nice_limits(float(np.nanmin(Y)), float(np.nanmax(Y)), pad=10.0)
    zlo, zhi = _nice_limits(float(np.nanmin(Z)), float(np.nanmax(Z)), pad=10.0)
    ax3d.set_xlim(xlo, xhi)
    ax3d.set_ylim(ylo, yhi)
    ax3d.set_zlim(zlo, zhi)
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Alt (m)")
    ax3d.view_init(elev=25, azim=-45)

    # Right-side traces + cursors
    ax_alt.plot(t, alt, linewidth=1.5, alpha=0.8)
    ax_alt.set_title("Altitude (m)")
    ax_alt.set_xlabel("t (s)")
    ax_alt.set_ylabel("m")
    alt_cursor = ax_alt.axvline(t[0], linestyle="--")

    ax_tas.plot(t, tas, linewidth=1.5, alpha=0.8)
    ax_tas.set_title("Airspeed (m/s)")
    ax_tas.set_xlabel("t (s)")
    ax_tas.set_ylabel("m/s")
    tas_cursor = ax_tas.axvline(t[0], linestyle="--")

    ax_rates.plot(t, p, label="p (rad/s)", linewidth=1.2, alpha=0.9)
    ax_rates.plot(t, q, label="q (rad/s)", linewidth=1.2, alpha=0.9)
    ax_rates.plot(t, r, label="r (rad/s)", linewidth=1.2, alpha=0.9)
    ax_rates.set_title("Body Rates (rad/s)")
    ax_rates.set_xlabel("t (s)")
    ax_rates.legend(loc="upper right", fontsize=8)
    rates_cursor = ax_rates.axvline(t[0], linestyle="--")

    ax_euler.plot(t, roll_deg, label="roll φ (deg)", linewidth=1.2, alpha=0.9)
    ax_euler.plot(t, pitch_deg, label="pitch θ (deg)", linewidth=1.2, alpha=0.9)
    ax_euler.plot(t, yaw_deg, label="yaw ψ (deg)", linewidth=1.2, alpha=0.9)
    ax_euler.set_title("Euler Angles (deg)")
    ax_euler.set_xlabel("t (s)")
    ax_euler.legend(loc="upper right", fontsize=8)
    euler_cursor = ax_euler.axvline(t[0], linestyle="--")

    for ax in (ax_alt, ax_tas, ax_rates, ax_euler):
        ax.set_xlim(t[0], t[-1])

    # -------- Precompute per-frame rotations for speed --------
    # We only compute for frames we will actually render (idx_anim).
    R_list = [R_from_euler_zyx(roll[k], pitch[k], yaw[k]) for k in idx_anim]

    # Update function
    def update(i: int):
        k = idx_anim[i]

        # track
        track_live.set_data(X[: k + 1], Y[: k + 1])
        track_live.set_3d_properties(Z[: k + 1])

        # body
        R = R_list[i]
        # rotate body points (display frame) into world, then translate to current pos
        rotated = body_pts_disp @ R.T
        Pw = np.column_stack(
            (X[k] + rotated[:, 0], Y[k] + rotated[:, 1], Z[k] + rotated[:, 2])
        )
        for line, i0, i1 in body_lines:
            line.set_data([Pw[i0, 0], Pw[i1, 0]], [Pw[i0, 1], Pw[i1, 1]])
            line.set_3d_properties([Pw[i0, 2], Pw[i1, 2]])

        # cursors
        x_t = t[k]
        alt_cursor.set_xdata([x_t, x_t])
        tas_cursor.set_xdata([x_t, x_t])
        rates_cursor.set_xdata([x_t, x_t])
        euler_cursor.set_xdata([x_t, x_t])

        # Return updated artists (no blit for 3D)
        return [track_live, alt_cursor, tas_cursor, rates_cursor, euler_cursor] + [
            ln for ln, _, _ in body_lines
        ]

    global _anim_ref
    _anim_ref = FuncAnimation(
        fig,
        update,
        frames=len(idx_anim),
        interval=interval_ms,
        blit=False,
        repeat=False,
    )
    plt.show()
    return _anim_ref
