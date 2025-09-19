import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass
from utils.location import WorldFrameCoordinate


@dataclass
class ThermalEstimate:
    W0: float
    Rth: float
    est_core: WorldFrameCoordinate
    confidence: float = 0.0

    average_actual_thermal_strength: float = (
        0.0  # The actual average strength of the thermal from variometer samples
    )

    def get_strength(self):
        return self.W0

    def get_radius(self):
        return self.Rth

    def get_location(self):
        return self.est_core

    def get_average_sampled_thermal_strength(self):
        return self.average_actual_thermal_strength


class ReducedOrderGaussianThermalModel:
    def __init__(self, thermal_estimate: ThermalEstimate):
        self.params = thermal_estimate

    def thermal_model_estimate_updraft(self, aircraft_location):
        # aircraft_location and thermal_location are WorldFrameCoordinate
        r = aircraft_location.distance_to(self.params.est_core)
        base = self.params.W0
        exponent = -1.0 * (r / self.params.Rth) ** 2
        w = base * np.exp(exponent)
        return w


class ThermalEstimator:
    def __init__(self, num_samples_to_buffer, variometer_noise_std=0.5, debug=False):
        self.num_samples_to_buffer = num_samples_to_buffer
        self.variometer_noise_std = variometer_noise_std
        self.samples = []
        self.estimate = ThermalEstimate(1.0, 50.0, WorldFrameCoordinate(0.0, 0.0), 0.0)
        self.prev_params = [
            self.estimate.W0,
            self.estimate.Rth,
            self.estimate.est_core.x,
            self.estimate.est_core.y,
        ]
        self.no_thermal_lock_threshold = 0.5  # m/s
        self.debug = debug

        self.lambda1 = 1.0
        self.lambda2 = 1.0
        self.lambda3 = 1.0

        # Confidence-based scaling - can adapt lambda based on how good the fit is.
        self.lambda_multiplier = 0.2
        self.confidence = 0.0
        self.average_sample_thermal_strength = 0.0

    def step(self, measurement, location):
        # location is now WorldFrameCoordinate
        assert hasattr(location, "x") and hasattr(
            location, "y"
        ), "Location must be a WorldFrameCoordinate."
        self.samples.append((measurement, location))
        if len(self.samples) > self.num_samples_to_buffer:
            self.samples.pop(0)
            self.average_sample_thermal_strength = np.mean([s[0] for s in self.samples])

        self.prev_params = [
            self.estimate.W0,
            self.estimate.Rth,
            self.estimate.est_core.x,
            self.estimate.est_core.y,
        ]

        def cost_function(params):
            W0, Rth, x_c, y_c = params
            total_error = 0.0
            core = WorldFrameCoordinate(x_c, y_c)
            for meas, loc in self.samples:
                eval_params = ThermalEstimate(W0, Rth, core)

                gaussian_model = ReducedOrderGaussianThermalModel(eval_params)

                predicted_w = gaussian_model.thermal_model_estimate_updraft(loc)
                total_error += (meas - predicted_w) ** 2

            W0_prev, Rth_prev, x_prev, y_prev = self.prev_params
            reg_W0 = (W0 - W0_prev) ** 2
            reg_Rth = (Rth - Rth_prev) ** 2
            reg_xy = (x_c - x_prev) ** 2 + (y_c - y_prev) ** 2

            lam = self.lambda_multiplier
            reg = lam * (
                self.lambda1 * reg_W0 + self.lambda2 * reg_Rth + self.lambda3 * reg_xy
            )

            return total_error + reg

        initial_guess = [
            self.estimate.W0,
            self.estimate.Rth,
            self.estimate.est_core.x,
            self.estimate.est_core.y,
        ]
        if self.average_sample_thermal_strength < self.no_thermal_lock_threshold:
            initial_guess[0] = 1.0
            initial_guess[1] = 50.0
            initial_guess[2] = location.x
            initial_guess[3] = location.y

        bounds = [(0.1, 20.0), (1.0, 200.0), (None, None), (None, None)]
        result = minimize(
            cost_function, initial_guess, bounds=bounds, method="Nelder-Mead"
        )
        if result.success:
            W0, Rth, x_c, y_c = result.x
            self.estimate.W0 = W0
            self.estimate.Rth = Rth
            self.estimate.est_core = WorldFrameCoordinate(x_c, y_c)
            if self.debug:
                print(
                    f"Estimated Parameters: W0={W0:.2f}, Rth={Rth:.2f}, x_c={x_c:.2f}, y_c={y_c:.2f}"
                )
        else:
            print("Optimization failed:", result.message)

        self.update_confidence()
        self.estimate.confidence = self.confidence
        self.estimate.average_actual_thermal_strength = (
            self.average_sample_thermal_strength
        )
        return self.estimate

    def update_confidence(self):
        # This could be tuned better...
        if len(self.samples) < self.num_samples_to_buffer / 10:
            self.confidence = 0.0
            return

        errors = []
        W0, Rth = self.estimate.W0, self.estimate.Rth
        core = self.estimate.est_core
        errors = []
        for meas, loc in self.samples:
            gaussian_model = ReducedOrderGaussianThermalModel(
                ThermalEstimate(W0, Rth, core)
            )
            predicted_w = gaussian_model.thermal_model_estimate_updraft(loc)

            errors.append(meas - predicted_w)
        errors = np.array(errors)

        # Chi-squared test of significane. Basically asking how likely is it that the
        # observed errors are due to measurement noise alone.
        chi2 = np.mean(errors**2) / (self.variometer_noise_std**2)
        self.confidence = 1 / (1 + chi2)
        # self.lambda_multiplier = self.confidence

    def get_estimate(self):
        """
        Returns:
            ThermalEstimate: Estimated thermal parameters and confidence.
        """
        return self.estimate

    def eval_thermal_updraft_with_estimated_params(
        self, radius_to_center, thermal_estimate: ThermalEstimate
    ):
        eval_coordinate = WorldFrameCoordinate(radius_to_center, 0.0)
        core = WorldFrameCoordinate(0.0, 0.0)
        return self.thermal_model_estimate_updraft(
            eval_coordinate, core, thermal_estimate.W0, thermal_estimate.Rth
        )

    def get_average_thermal_strength(self):
        return self.average_sample_thermal_strength
