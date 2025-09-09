import numpy as np
import sympy as sp


class ThermalEstimator:
    def __init__(self, initial_covariance, process_noise, measurement_noise):
        # State: [x_c, y_c, W_0, R_th]^T
        self.state = np.array([0.0, 0.0, 1.0, 10.0], dtype=float).reshape(-1, 1)  # (4,1)

        # Noises & covariance
        self.covariance = np.array(initial_covariance, dtype=float)
        self.process_noise = np.array(process_noise, dtype=float)
        self.measurement_noise = np.array(measurement_noise, dtype=float)

        # Shape checks
        assert self.covariance.shape == (4, 4), "Covariance must be a 4x4 matrix."
        assert self.process_noise.shape == (4, 4), "Process noise must be a 4x4 matrix."
        assert self.measurement_noise.shape == (1, 1), "Measurement noise must be a 1x1 matrix."

        # Definiteness checks
        assert np.all(np.linalg.eigvals(self.covariance) > 0), "Covariance must be positive definite."
        assert np.all(np.linalg.eigvals(self.process_noise) >= 0), "Process noise must be positive semi-definite."
        assert np.all(np.linalg.eigvals(self.measurement_noise) >= 0), "Measurement noise must be positive semi-definite."

        self._init_symbolic_jacobians()

    def _init_symbolic_jacobians(self):
        # Symbols
        x_c, y_c, W_0, R_th = sp.symbols('x_c y_c W_0 R_th')
        x_m, y_m = sp.symbols('x_m y_m')  # control inputs (glider position)
        # w_meas is the measured vertical air mass velocity (variometer), scalar

        # State & control vectors (for clarity)
        state_vector = sp.Matrix([x_c, y_c, W_0, R_th])
        # control_vector = sp.Matrix([x_m, y_m])  # not used directly in lambdify

        # Radial distance^2 from thermal center
        radial_error_squared = (x_m - x_c) ** 2 + (y_m - y_c) ** 2

        # Simple exponential-bell thermal model: w = W0 * exp(-(r^2 / R_th^2))
        rho = -(radial_error_squared) / R_th ** 2
        w_meas_expected = W_0 * sp.exp(rho)

        # Measurement Jacobian H = d h(x,u) / d x evaluated at current state (1x4)
        H = sp.Matrix([[sp.diff(w_meas_expected, var) for var in state_vector]])
        H = sp.simplify(H)

        # Fast numerical function: inputs (x_c, y_c, W_0, R_th, x_m, y_m) -> H (1x4)
        self.H_jacobian_func = sp.lambdify((x_c, y_c, W_0, R_th, x_m, y_m), H, 'numpy')

        # Constant model => F = I
        self.F_jacobian = np.eye(4)

    def predict(self):
        # State prediction (constant model): x_k|k-1 = x_k-1|k-1
        # (No change to self.state)
        # Covariance prediction: P = F P F^T + Q
        self.covariance = self.F_jacobian @ self.covariance @ self.F_jacobian.T + self.process_noise
        # (Optional) Symmetrize to avoid numerical asymmetry
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

    def update(self, measurement, control_input):
        """
        measurement: scalar float (w_meas)
        control_input: tuple/list (x_m, y_m) giving the glider's x,y position where the measurement was taken
        """
        x_m, y_m = control_input
        x_c, y_c, W_0, R_th = self.state.flatten()

        # Expected measurement h(x,u)
        radial_error_squared = (x_m - x_c) ** 2 + (y_m - y_c) ** 2
        rho = -(radial_error_squared) / (R_th ** 2)
        w_meas_expected = W_0 * np.exp(rho)

        # Innovation (as 1x1 array to keep matrix math consistent)
        y_tilde = np.array([[float(measurement) - float(w_meas_expected)]], dtype=float)  # (1,1)

        # H (1x4)
        H = np.array(self.H_jacobian_func(x_c, y_c, W_0, R_th, x_m, y_m), dtype=float)  # (1,4)

        # Innovation covariance S (1x1)
        S = H @ self.covariance @ H.T + self.measurement_noise  # (1,1)

        # Kalman Gain K (4x1)
        K = self.covariance @ H.T @ np.linalg.inv(S)  # (4,1)

        # State update: x = x + K * y
        self.state = self.state + K @ y_tilde  # (4,1)

        # Covariance update (standard): P = (I - K H) P
        I = np.eye(4)
        self.covariance = (I - K @ H) @ self.covariance
        # Symmetrize to control numerical drift
        self.covariance = 0.5 * (self.covariance + self.covariance.T)

        # Checks
        assert self.state.shape == (4, 1), "State vector must be (4,1)."
        assert self.covariance.shape == (4, 4), "Covariance must be a 4x4 matrix."
        # Positive semi-definiteness is typical after the update; allow small negative due to numeric error
        eigvals = np.linalg.eigvals(self.covariance)
        assert np.all(np.real(eigvals) > -1e-9), "Updated covariance must be positive semi-definite."

    # Optional: expose current estimate neatly
    def get_state(self):
        return {
            "x_c": float(self.state[0, 0]),
            "y_c": float(self.state[1, 0]),
            "W_0": float(self.state[2, 0]),
            "R_th": float(self.state[3, 0]),
        }


if __name__ == "__main__":
    # Example usage with consistent dimensions
    initial_cov = np.eye(4) * 0.1            # (4x4)
    process_noise = np.eye(4) * 0.01         # (4x4)
    measurement_noise = np.array([[0.05]])   # (1x1) ← scalar measurement

    estimator = ThermalEstimator(initial_cov, process_noise, measurement_noise)

    # Dummy predict/update cycle
    estimator.predict()
    # Suppose the glider is at (x_m, y_m) and we measured w_meas
    x_m, y_m = 10.0, -3.0
    w_meas = 0.8
    estimator.update(w_meas, (x_m, y_m))

    print("State estimate:", estimator.get_state())
    print("Covariance:\n", estimator.covariance)
