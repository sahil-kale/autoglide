import numpy as np
from scipy.optimize import minimize

class ThermalEstimator:
    def __init__(self, num_samples_to_buffer, debug=False):
        self.num_samples_to_buffer = num_samples_to_buffer
        self.samples = []
        self.estimated_params = [1.0, 50.0, 0.0, 0.0]  # Initial guess: [W0, Rth, x_th, y_th]
        self.prev_params = self.estimated_params.copy()
        self.no_thermal_lock_threshold = 0.9  # m/s
        self.debug = debug

        self.lambda1 = 1.0
        self.lambda2 = 1.0
        self.lambda3 = 1.0

        # Confidence-based scaling - can adapt lambda based on how good the fit is.
        self.lambda_multiplier = 1.0

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

        self.prev_params = self.estimated_params.copy()
        
        def cost_function(params):
            W0, Rth, x_c, y_c = params
            total_error = 0.0
            for meas, loc in self.samples:
                predicted_w = self.thermal_model_estimate_updraft(loc, (x_c, y_c), W0, Rth)
                total_error += (meas - predicted_w) ** 2

            W0_prev, Rth_prev, x_prev, y_prev = self.prev_params
            reg_W0  = (W0  - W0_prev)**2
            reg_Rth = (Rth - Rth_prev)**2
            reg_xy  = (x_c - x_prev)**2 + (y_c - y_prev)**2

            lam = self.lambda_multiplier
            reg = lam * (self.lambda1 * reg_W0 + self.lambda2 * reg_Rth + self.lambda3 * reg_xy)

            return total_error + reg
        
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
            if self.debug:
                print(f"Estimated Parameters: W0={self.estimated_params[0]:.2f}, Rth={self.estimated_params[1]:.2f}, x_th={self.estimated_params[2]:.2f}, y_th={self.estimated_params[3]:.2f}")
        else:
            print("Optimization failed:", result.message)

        self.update_confidence()

        return self.estimated_params
    
    def update_confidence(self):
        if (len(self.samples) < self.num_samples_to_buffer):
            return 0.0
        
        errors = []
        W0, Rth, x_th, y_th = self.estimated_params
        for meas, loc in self.samples:
            predicted_w = self.thermal_model_estimate_updraft(loc, (x_th, y_th), W0, Rth)
            errors.append(meas - predicted_w)
        
        errors = np.array(errors)
        mean_error = np.mean(errors)
        variance_error = np.var(errors)

        confidence = 1.0 / (1.0 + variance_error)
        self.lambda_multiplier = confidence
