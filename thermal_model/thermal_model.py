import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (import registers 3D projection)

class ThermalModel:
    def __init__(self, w_max, r_th, x_th, y_th, V_e):
        self.w_max = w_max  # Maximum thermal uplift (m/s)
        self.r_th = r_th    # Thermal radius (m)
        self.x_th = x_th    # Thermal center x position (m)
        self.y_th = y_th    # Thermal center y position (m)
        self.V_e = V_e      # Base sink rate (m/s), positive downwards

        assert self.w_max > 0, "w_max must be positive."
        assert self.r_th > 1, "r_th must be positive and greater than 1."
        assert self.V_e >= 0, "V_e must be non-negative."

    def get_thermal_uplift(self, x, y, h):
        r = np.sqrt((x - self.x_th) ** 2 + (y - self.y_th) ** 2)
        base = self.w_max + self.V_e
        exponent = -1.0 * (r/self.r_th) ** 2
        w = base * np.exp(exponent) - self.V_e
        return w


if __name__ == "__main__":
    thermal = ThermalModel(w_max=5.0, r_th=50.0, x_th=0.0, y_th=0.0, V_e=1.0)

    # grid
    x = np.linspace(-150, 150, 100)
    y = np.linspace(-150, 150, 100)
    X, Y = np.meshgrid(x, y)
    Z = thermal.get_thermal_uplift(X, Y, h=100.0)

    # 3D plot
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection="3d")
    surf = ax.plot_surface(X, Y, Z, cmap="viridis", edgecolor="none", alpha=0.9)

    # labels and formatting
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Thermal Uplift (m/s)")
    ax.set_title("3D Thermal Uplift Field")
    fig.colorbar(surf, ax=ax, shrink=0.6, aspect=12, label="Thermal Uplift (m/s)")

    # mark thermal center
    ax.scatter(thermal.x_th, thermal.y_th, thermal.w_max, color="red", marker="x", s=100, label="Thermal Center")
    ax.legend()

    plt.show()
