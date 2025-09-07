import numpy as np

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
        """
        Compute the thermal uplift at a given position.

        Args:
            x: x position (m)
            y: y position (m)
            h: altitude (m) - not used in current model but could be in future extensions

        Returns:
            Thermal uplift (m/s), positive upwards
        """
        r = np.sqrt((x - self.x_th) ** 2 + (y - self.y_th) ** 2)
        base = self.w_max + self.V_e
        exponent = -1.0 * (r/self.r_th) ** 2
        w = base * np.exp(exponent) - self.V_e
        return w

if __name__ == "__main__":
    # Example usage
    thermal = ThermalModel(w_max=5.0, r_th=50.0, x_th=0.0, y_th=0.0, V_e=1.0)

    import matplotlib.pyplot as plt
    # plot a 2D heatmap of the thermal uplift
    x = np.linspace(-150, 150, 300)
    y = np.linspace(-150, 150, 300)
    X, Y = np.meshgrid(x, y)
    Z = thermal.get_thermal_uplift(X, Y, h=100.0)
    plt.figure(figsize=(8, 6))
    cp = plt.contourf(X, Y, Z, levels=50, cmap='viridis')
    plt.colorbar(cp, label='Thermal Uplift (m/s)')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Thermal Uplift Heatmap')
    plt.scatter(thermal.x_th, thermal.y_th, color='red', marker='x', label='Thermal Center')
    plt.legend()
    plt.show()
