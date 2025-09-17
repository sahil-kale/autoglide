import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (import registers 3D projection)
from dataclasses import dataclass


# --- Parameterization for Monte-Carlo Simulations ---
@dataclass
class ThermalModelParams:
    w_max: float = 7.0
    r_th: float = 70.0
    x_th: float = 100.0
    y_th: float = 50.0
    V_e: float = 1.0
    kx: float = 0.03
    ky: float = -0.02
    core_center_random_noise_std: float = 1.5


class ThermalModel:
    def __init__(
        self,
        params: "ThermalModelParams" = None,
    ):
        assert params is not None, "ThermalModelParams must be provided."

        self.w_max = params.w_max
        self.r_th = params.r_th
        self.x_th = params.x_th
        self.y_th = params.y_th
        self.V_e = params.V_e
        self.kx = params.kx
        self.ky = params.ky
        self.core_center_random_noise_std = params.core_center_random_noise_std

        self.core_center_random_noise_mean = 0.0  # always zero mean

        assert self.w_max is not None and self.w_max > 0, "w_max must be positive."
        assert (
            self.r_th is not None and self.r_th > 1
        ), "r_th must be positive and greater than 1."
        assert self.V_e is not None and self.V_e >= 0, "V_e must be non-negative."

    def core_center_at_height(self, h):
        # Linear drift of core center with altitude
        # could incorporate wind shear model but keeping it simple for now
        eta_x = np.random.normal(
            self.core_center_random_noise_mean, self.core_center_random_noise_std
        )
        eta_y = np.random.normal(
            self.core_center_random_noise_mean, self.core_center_random_noise_std
        )
        x_c = self.x_th + self.kx * h + eta_x
        y_c = self.y_th + self.ky * h + eta_y
        return x_c, y_c

    def get_thermal_uplift(self, x, y, h):
        x_c, y_c = self.core_center_at_height(h)
        r = np.sqrt((x - x_c) ** 2 + (y - y_c) ** 2)
        base = self.w_max + self.V_e
        exponent = -1.0 * (r / self.r_th) ** 2
        w = base * np.exp(exponent) - self.V_e
        return w


if __name__ == "__main__":
    # Example with slight tilt: 0.03 m drift in x and -0.02 m in y per meter climb
    thermal = ThermalModel(
        w_max=5.0,
        r_th=50.0,
        x_th=0.0,
        y_th=0.0,
        V_e=1.0,
        kx=0.03,
        ky=-0.02,
        core_center_random_noise_std=1.5,
    )

    # grid
    x = np.linspace(-150, 150, 100)
    y = np.linspace(-150, 150, 100)
    X, Y = np.meshgrid(x, y)
    h_plot = 100.0
    Z = thermal.get_thermal_uplift(X, Y, h=h_plot)

    # 3D surface of uplift at a fixed altitude
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(X, Y, Z, cmap="viridis", edgecolor="none", alpha=0.9)

    # labels and formatting
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Thermal Uplift (m/s)")
    ax.set_title(f"3D Thermal Uplift Field (h = {h_plot:.0f} m)")
    fig.colorbar(surf, ax=ax, shrink=0.6, aspect=12, label="Thermal Uplift (m/s)")

    # mark thermal center at this altitude
    xc, yc = thermal.core_center_at_height(h_plot)
    ax.scatter(
        xc, yc, thermal.w_max, color="red", marker="x", s=100, label="Thermal Center"
    )
    ax.legend()

    # --- New plot: core drift with altitude (1/2 * R_th rings) ---
    # This plot uses Z-axis as altitude to visualize the tilted column.
    fig2 = plt.figure(figsize=(10, 7))
    ax2 = fig2.add_subplot(111, projection="3d")

    # Choose altitudes to visualize
    hs = np.linspace(0.0, 600.0, 10)  # meters
    r_ring = 0.5 * thermal.r_th
    theta = np.linspace(0, 2 * np.pi, 200)

    # Plot core center path
    centers_x = []
    centers_y = []
    for h in hs:
        x_c, y_c = thermal.core_center_at_height(h)
        centers_x.append(x_c)
        centers_y.append(y_c)

        # ring at radius r_ring around core center at altitude h
        xr = x_c + r_ring * np.cos(theta)
        yr = y_c + r_ring * np.sin(theta)
        zr = np.full_like(theta, h)
        ax2.plot(xr, yr, zr, linewidth=1.5)

    ax2.plot(centers_x, centers_y, hs, "r--", linewidth=2, label="Core center path")

    ax2.set_xlabel("X Position (m)")
    ax2.set_ylabel("Y Position (m)")
    ax2.set_zlabel("Altitude h (m)")
    ax2.set_title("Thermal Core Drift with Altitude (½·R_th Rings)")
    ax2.legend()

    plt.show()
