import numpy as np
from utils.first_order_filter import first_order_filter
from utils.constants import GRAVITY_M_PER_S_SQ


class GliderModelParams:
    def __init__(
        self, V_star, s_min, k_v, alpha_n, initial_altitude, roll_tau, vel_tau
    ):
        self.V_star = V_star  # Best glide speed (m/s)
        self.s_min = s_min  # Minimum sink rate (m/s)
        self.k_v = k_v  # Sink rate coefficient ((m/s)/(m/s)^2)
        self.alpha_n = alpha_n  # Load factor exponent (dimensionless)
        self.initial_altitude = initial_altitude  # Initial altitude (m)

        self.vel_tau = vel_tau  # Airspeed dynamics time constant (s)
        self.roll_tau = roll_tau  # Roll dynamics time constant (s)

        assert self.V_star > 0, "V_star must be positive."
        assert self.s_min > 0, "s_min must be positive."
        assert self.k_v > 0, "k_v must be positive."
        assert self.alpha_n >= 0, "alpha_n must be non-negative."

    def get_sink_rate(self, V, phi):
        s0 = (
            self.s_min + self.k_v * (V - self.V_star) ** 2
        )  # Sink rate at zero bank angle
        n = 1 / np.cos(phi)  # Load factor
        return s0 * (n**self.alpha_n)  # Adjusted sink rate for bank angle


class GliderKinematicModelControl:
    def __init__(self, roll_angle, airspeed):
        self.phi = roll_angle  # Roll angle (rad), CCW positive
        self.V = airspeed  # Airspeed (m/s)


class GliderKinematicModelDisturbance:
    def __init__(self, thermal_uplift=0.0, wind_x=0.0, wind_y=0.0):
        self.w = thermal_uplift  # Thermal uplift (m/s), positive upwards
        self.W_x = wind_x  # Wind in x direction (m/s)
        self.W_y = wind_y  # Wind in y direction (m/s)


class GliderOpenLoopKinematicModel:
    def __init__(self, params: GliderModelParams):
        assert params is not None, "Parameters must be provided."
        self.params = params

        # state variables
        self.x = 0.0  # Position in x in world frame (m)
        self.y = 0.0  # Position in y in world frame (m)
        self.h = params.initial_altitude  # Altitude (m)
        self.psi = 0.0  # Heading (rad), 0 = West, CCW positive
        self.phi = 0.0  # Roll angle (rad), CCW positive
        self.V = params.s_min  # Airspeed (m/s)

    def step(
        self,
        dt,
        control: GliderKinematicModelControl,
        disturbance: GliderKinematicModelDisturbance,
    ):
        assert dt > 0, "Time step must be positive."
        assert control is not None, "Control input must be provided."
        assert disturbance is not None, "Disturbance input must be provided."

        # Update roll angle and airspeed using first order filter
        self.phi = first_order_filter(self.phi, control.phi, self.params.roll_tau, dt)
        self.V = first_order_filter(self.V, control.V, self.params.vel_tau, dt)

        # Propogate the state
        xdot = self.V * np.cos(self.psi) + disturbance.W_x
        ydot = self.V * np.sin(self.psi) + disturbance.W_y
        psi_dot = (
            GRAVITY_M_PER_S_SQ / self.V * np.tan(self.phi)
        )  # turn-coordinated kinematic model
        h_dot = disturbance.w - self.params.get_sink_rate(self.V, self.phi)

        # Euler integration, keep it chill
        self.x += xdot * dt
        self.y += ydot * dt
        self.psi += psi_dot * dt
        self.h += h_dot * dt
