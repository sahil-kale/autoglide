from utils.pid import PIDController, PIDConfig


def test_pid_windup_simple():
    gains = PIDConfig(
        kp=1.0, ki=1.0, kd=0.0, actuator_limit_high=10.0, actuator_limit_low=-5.0
    )
    pid = PIDController(gains, dt=1.0)

    output = pid.step(1.0, 0.0)
    assert output == 2.0  # 1 (P) + 1 (I)

    output = pid.step(1.0, 0.0)
    assert output == 3.0  # 1 (P) + 2 (I)

    pid.reset()

    output = pid.step(9.5, 0.0)
    assert output == 10.0  # limited to actuator_limit_high
