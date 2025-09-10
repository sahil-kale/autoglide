import numpy as np
from scipy.optimize import minimize

class ThermalEstimator:
    def __init__(self, num_samples_to_buffer):
        self.num_samples_to_buffer = num_samples_to_buffer
        self.samples = []
        self.estimated_params = [1.0, 50.0, 0.0, 0.0]  # Initial guess: [W0, Rth, x_th, y_th]
        self.no_thermal_lock_threshold = 0.9  # m/s

    def thermal_model_estimate_updraft(self, aircraft_location, thermal_location, W0, Rth):
        x, y = aircraft_location
        x_th, y_th = thermal_location
        r = np.sqrt((x - x_th) ** 2 + (y - y_th) ** 2)
        base = W0
        exponent = -1.0 * (r/Rth) ** 2
        w = base * np.exp(exponent)
        return w

    def step(self, measurement, location):
        assert location.shape == (2,), "Location must be a 2D coordinate (x, y)."
        self.samples.append((measurement, location))
        if len(self.samples) > self.num_samples_to_buffer:
            self.samples.pop(0)
        
        def cost_function(params):
            W0, Rth, x_c, y_c = params
            total_error = 0.0
            for meas, loc in self.samples:
                predicted_w = self.thermal_model_estimate_updraft(loc, (x_c, y_c), W0, Rth)
                total_error += (meas - predicted_w) ** 2
            return total_error
        
        initial_guess = self.estimated_params
        if self.estimated_params[0] < self.no_thermal_lock_threshold:
            initial_guess[0] = 1.0
            initial_guess[1] = 50.0
            initial_guess[2] = location[0]
            initial_guess[3] = location[1]

        bounds = [(0.1, 20.0), (1.0, 200.0), (None, None), (None, None)]
        result = minimize(cost_function, initial_guess, bounds=bounds, method='Nelder-Mead')
        if result.success:
            self.estimated_params = result.x
            print(f"Estimated Parameters: W0={self.estimated_params[0]:.2f}, Rth={self.estimated_params[1]:.2f}, x_th={self.estimated_params[2]:.2f}, y_th={self.estimated_params[3]:.2f}")
        else:
            print("Optimization failed:", result.message)

        return self.estimated_params
