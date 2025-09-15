import numpy as np
from mpl_toolkits.mplot3d.art3d import Line3DCollection

def airplane_segments(x: float, y: float, z: float, psi: float, phi: float, scale: float):
    cpsi, spsi = np.cos(psi), np.sin(psi)
    ex = np.array([cpsi, spsi, 0.0])
    ey = np.array([-spsi, cpsi, 0.0])
    ez = np.array([0.0, 0.0, 1.0])
    p0 = np.array([x, y, z])
    nose = p0 + ex * (scale * 0.8)
    tail = p0 - ex * (scale * 0.7)
    wz = np.tan(phi) * scale * 0.5
    wing_r = p0 + ey * (scale * 0.6) - ez * wz
    wing_l = p0 - ey * (scale * 0.6) + ez * wz
    tail_r = tail + ey * (scale * 0.25)
    tail_l = tail - ey * (scale * 0.25)
    segs = [
        (tail, nose),
        (wing_l, wing_r),
        (tail_l, tail_r),
    ]
    return segs, nose, ex, ey, ez

def draw_airplane(ax, x, y, z, psi, phi, scale):
    segs, nose, ex, ey, ez = airplane_segments(x, y, z, psi, phi, scale)
    lc = Line3DCollection(segs, linewidths=2.0, colors="red")
    ax.add_collection3d(lc)
    ax.plot(
        [nose[0], nose[0] - 0.2 * ex[0] + 0.15 * ey[0], nose[0] - 0.2 * ex[0] - 0.15 * ey[0], nose[0]],
        [nose[1], nose[1] - 0.2 * ex[1] + 0.15 * ey[1], nose[1] - 0.2 * ex[1] - 0.15 * ey[1], nose[1]],
        [nose[2], nose[2] - 0.2 * ex[2] + 0.15 * ey[2], nose[2] - 0.2 * ex[2] - 0.15 * ey[2], nose[2]],
        linewidth=1.2,
    )
