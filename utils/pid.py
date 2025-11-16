class PIDConfig:
    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        actuator_limit_high: float = None,
        actuator_limit_low: float = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.actuator_limit_high = actuator_limit_high
        self.actuator_limit_low = actuator_limit_low


class PIDController:
    def __init__(self, config: PIDConfig, dt: float):
        self.config = config
        self.dt = dt
        self.reset()

    def reset(self):
        self.integral_error = 0.0
        self.prev_error = 0.0

    def step(self, target: float, current: float) -> float:
        error = target - current

        proportional_term = self.config.kp * error
        self.integral_error += error * self.dt  # Right-hand rule integration

        if self.config.actuator_limit_high is not None:
            permissible_integral_authority = (
                self.config.actuator_limit_high - proportional_term
            ) / self.config.ki
            self.integral_error = min(
                self.integral_error, permissible_integral_authority
            )

        if self.config.actuator_limit_low is not None:
            permissible_integral_authority = (
                self.config.actuator_limit_low - proportional_term
            ) / self.config.ki
            self.integral_error = max(
                self.integral_error, permissible_integral_authority
            )

        derivative_error = (error - self.prev_error) / self.dt
        self.prev_error = error

        control_output = (
            proportional_term
            + self.config.ki * self.integral_error
            + self.config.kd * derivative_error
        )
        return control_output
