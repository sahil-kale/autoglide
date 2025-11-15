import math
from typing import List, Callable, Tuple
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.animation import FuncAnimation

_anim_ref = None  # prevent GC


# ---------- utilities ----------
def _nice_limits(vmin: float, vmax: float, pad: float = 10.0) -> Tuple[float, float]:
    if not np.isfinite(vmin) or not np.isfinite(vmax):
        return -1.0, 1.0
    if abs(vmax - vmin) < 1e-6:
        c = 0.5 * (vmax + vmin)
        return c - pad, c + pad
    return vmin - pad, vmax + pad


def latlon_to_local_NE(
    lat_deg: np.ndarray, lon_deg: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Local tangent plane in meters (origin = first sample): returns (N, E)."""
    if len(lat_deg) == 0:
        return np.array([]), np.array([])
    lat0 = math.radians(float(lat_deg[0]))
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(lat0)
    N = (lat_deg - lat_deg[0]) * m_per_deg_lat
    E = (lon_deg - lon_deg[0]) * m_per_deg_lon
    return N, E


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
    # fin (+Z down)
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


def R_body_to_NED_from_euler_zyx(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Body → NED rotation using aerospace (ZYX) convention."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _set_axes_equal_3d(ax):
    """Equal scale on all 3 axes so the aircraft isn’t distorted."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    span = limits[:, 1] - limits[:, 0]
    centers = np.mean(limits, axis=1)
    max_range = max(span)
    for ctr, setter in zip(centers, [ax.set_xlim3d, ax.set_ylim3d, ax.set_zlim3d]):
        setter([ctr - max_range / 2, ctr + max_range / 2])


# ---------- main ----------
def animate_sim(
    states: List,
    *,
    euler_from_state: Callable[
        [object], Tuple[float, float, float]
    ] = lambda s: s.attitude.get_euler(),
    interval_ms: int = 30,
    frame_step: int = 2,
    wire_scale: float = 5.0,
    view_elev: float = 25,
    view_azim: float = -45,  # tweak if you want a different oblique
):
    """
    Flight playback viewer (Display in E, N, Alt):
      - Display axes: X=East (m), Y=North (m), Z=Altitude (m, up)  [right-handed]
      - Data are still NED; we convert NED → (E, N, Alt) for plotting:
            x =  E
            y =  N
            z =  alt - ΔD
      - Body frame is +Z_down; rotations use aerospace ZYX (body→NED).
    """
    assert len(states) >= 2
    states = sorted(states, key=lambda s: float(s.time_s))

    # Series
    t = np.array([float(s.time_s) for s in states])
    alt = np.array([float(s.altitude_m) for s in states])
    tas = np.array([float(s.airspeed_mps) for s in states])
    p = np.array([float(s.p_radps) for s in states])
    q = np.array([float(s.q_radps) for s in states])
    r = np.array([float(s.r_radps) for s in states])
    lat = np.array([float(s.latitude_deg) for s in states])
    lon = np.array([float(s.longitude_deg) for s in states])

    # Position: compute N, E; display will be (E, N, Alt)
    N, E = latlon_to_local_NE(lat, lon)

    # Attitude
    eulers = np.array([euler_from_state(s) for s in states], dtype=float)
    roll, pitch, yaw = eulers[:, 0], eulers[:, 1], eulers[:, 2]
    roll_deg, pitch_deg, yaw_deg = np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    # Animation frames (decimated)
    idx_anim = np.arange(0, len(t), max(1, frame_step))
    R_list = [R_body_to_NED_from_euler_zyx(roll[k], pitch[k], yaw[k]) for k in idx_anim]

    # Figure
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
    fig.suptitle(
        "Flight Playback Viewer", fontsize=14
    )

    # 3D track in (E, N, Alt)
    (track_full,) = ax3d.plot(E, N, alt, linewidth=1, alpha=0.3)
    (track_live,) = ax3d.plot([], [], [], linewidth=2)

    # Wireframe + artists
    body_pts_b, body_segs = aircraft_wireframe(scale=wire_scale)
    body_lines = [
        (ax3d.plot([], [], [], linewidth=2)[0], i0, i1) for i0, i1 in body_segs
    ]

    # Limits (on displayed axes)
    elo, ehi = _nice_limits(float(np.nanmin(E)), float(np.nanmax(E)), pad=10.0)
    nlo, nhi = _nice_limits(float(np.nanmin(N)), float(np.nanmax(N)), pad=10.0)
    zlo, zhi = _nice_limits(float(np.nanmin(alt)), float(np.nanmax(alt)), pad=5.0)
    ax3d.set_xlim(elo, ehi)
    ax3d.set_ylim(nlo, nhi)
    ax3d.set_zlim(zlo, zhi)
    ax3d.set_xlabel("East (m)")
    ax3d.set_ylabel("North (m)")
    ax3d.set_zlabel("Altitude (m)")
    ax3d.view_init(elev=view_elev, azim=view_azim)
    _set_axes_equal_3d(ax3d)

    # Time traces
    ax_alt.plot(t, alt)
    ax_alt.set_title("Altitude (m)")
    ax_alt.set_xlabel("t (s)")
    ax_alt.set_ylabel("m")
    alt_cursor = ax_alt.axvline(t[0], linestyle="--")

    ax_tas.plot(t, tas)
    ax_tas.set_title("Airspeed (m/s)")
    ax_tas.set_xlabel("t (s)")
    ax_tas.set_ylabel("m/s")
    tas_cursor = ax_tas.axvline(t[0], linestyle="--")

    ax_rates.plot(t, p, label="p")
    ax_rates.plot(t, q, label="q")
    ax_rates.plot(t, r, label="r")
    ax_rates.legend()
    ax_rates.set_title("Body Rates (rad/s)")
    rates_cursor = ax_rates.axvline(t[0], linestyle="--")

    ax_euler.plot(t, roll_deg, label="φ")
    ax_euler.plot(t, pitch_deg, label="θ")
    ax_euler.plot(t, yaw_deg, label="ψ")
    ax_euler.legend()
    ax_euler.set_title("Euler Angles (deg)")
    euler_cursor = ax_euler.axvline(t[0], linestyle="--")

    for ax in (ax_alt, ax_tas, ax_rates, ax_euler):
        ax.set_xlim(t[0], t[-1])

    # ---- animation update ----
    def update(i: int):
        k = idx_anim[i]

        # Track (E, N, Alt)
        track_live.set_data(E[: k + 1], N[: k + 1])
        track_live.set_3d_properties(alt[: k + 1])

        # Body pose:
        # rotated (ΔN, ΔE, ΔD) in NED; display (E, N, Alt) = (ΔE, ΔN, alt - ΔD)
        R = R_list[i]
        rotated = body_pts_b @ R.T
        Pw = np.column_stack(
            (
                E[k] + rotated[:, 1],  # X = East
                N[k] + rotated[:, 0],  # Y = North
                alt[k] - rotated[:, 2],  # Z = Altitude
            )
        )
        for line, i0, i1 in body_lines:
            line.set_data([Pw[i0, 0], Pw[i1, 0]], [Pw[i0, 1], Pw[i1, 1]])
            line.set_3d_properties([Pw[i0, 2], Pw[i1, 2]])

        x_t = t[k]
        alt_cursor.set_xdata([x_t, x_t])
        tas_cursor.set_xdata([x_t, x_t])
        rates_cursor.set_xdata([x_t, x_t])
        euler_cursor.set_xdata([x_t, x_t])

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
