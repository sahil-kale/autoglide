import numpy as np
import sympy as sp

class ThermalEstimator:
    def __init__(self, initial_covariance, process_noise, measurement_noise):
        self.state = [0.0, 0.0, 0.0, 1.0] # [x_c, y_c, W_0, R_th]
        self.covariance = initial_covariance
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        assert self.covariance.shape == (4, 4), "Covariance must be a 4x4 matrix."
        assert self.process_noise.shape == (4, 4), "Process noise must be a 4x4 matrix."
        assert self.measurement_noise.shape == (2, 2), "Measurement noise must be a 2x2 matrix."
        assert np.all(np.linalg.eigvals(self.covariance) > 0), "Covariance must be positive definite."
        assert np.all(np.linalg.eigvals(self.process_noise) >= 0), "Process noise must be positive semi-definite."
        assert np.all(np.linalg.eigvals(self.measurement_noise) >= 0), "Measurement noise must be positive semi-definite."

        self._init_symbolic_jacobians()
    
    def _init_symbolic_jacobians(self):
        x_c, y_c, W_0, R_th = sp.symbols('x_c y_c W_0 R_th')
        x_m, y_m = sp.symbols('x_m y_m') # external control inputs (can be assumed as constants here)
        w_meas = sp.symbols('w_meas') # variometer input

        state_vector = sp.Matrix([x_c, y_c, W_0, R_th])
        control_vector = sp.Matrix([x_m, y_m, w_meas])
        radial_error_squared = (x_m - x_c)**2 + (y_m - y_c)**2
        rho = -(radial_error_squared)/R_th**2
        w_meas_expected = W_0 * sp.exp(rho)

        # calculate jacobin H based on measurement model
        H = sp.Matrix([[sp.diff(w_meas_expected, var) for var in state_vector]])
        H = sp.simplify(H)
        self.H_jacobian_func = sp.lambdify((x_c, y_c, W_0, R_th, x_m, y_m), H, 'numpy')
        # F is identity since we assume constant model
        self.F_jacobian = np.eye(4)

    def predict(self):
        # State prediction (constant model)
        self.state = self.state
        # Covariance prediction
        self.covariance = self.F_jacobian @ self.covariance @ self.F_jacobian.T + self.process_noise

    def update(self, measurement, control_input):
        x_m, y_m = control_input
        x_c, y_c, W_0, R_th = self.state

        # Compute the expected measurement
        radial_error_squared = (x_m - x_c)**2 + (y_m - y_c)**2
        rho = -(radial_error_squared)/R_th**2
        w_meas_expected = W_0 * np.exp(rho)

        # Measurement residual
        y_tilde = measurement - w_meas_expected

        # Compute the Jacobian H at the current state
        H = self.H_jacobian_func(x_c, y_c, W_0, R_th, x_m, y_m).astype(float)

        # Innovation covariance
        S = H @ self.covariance @ H.T + self.measurement_noise

        # Kalman Gain
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # State update
        self.state = self.state + (K @ y_tilde).flatten().tolist()

        # Covariance update
        I = np.eye(4)
        self.covariance = (I - K @ H) @ self.covariance

        assert np.all(np.linalg.eigvals(self.covariance) > 0), "Updated covariance must be positive definite."
        # ensure sizes are correct
        assert len(self.state) == 4, "State vector must have 4 elements."
        assert self.covariance.shape == (4, 4), "Covariance must be a 4x4 matrix."

if __name__ == "__main__":
    initial_cov = np.eye(4) * 0.1
    process_noise = np.eye(4) * 0.01
    measurement_noise = np.eye(2) * 0.05

    estimator = ThermalEstimator(initial_cov, process_noise, measurement_noise)    
