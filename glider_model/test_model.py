import numpy as np
import pytest
from glider_model.model import (
    GliderModelParams,
    GliderKinematicModelControl,
    GliderKinematicModelDisturbance,
    GliderOpenLoopKinematicModel,
)

DEFAULT_GLIDER_PARAMS = GliderModelParams(
    V_star=30,  # Best glide speed (m/s)
    V_stall=12,  # Stall speed (m/s)
    s_min=0.6,  # Minimum sink rate (m/s)
    k_v=0.01,  # Sink rate coefficient ((m/s)/(m/s)^2)
    alpha_n=0.7,  # Load factor exponent (dimensionless)
    initial_altitude=1000,  # Initial altitude (m)
    vel_tau=1.5,  # Airspeed dynamics time constant (s)
    roll_tau=2.0,  # Roll dynamics time constant (s)
    roll_rate_limit_rad_per_s=np.radians(30),  # rad/s
)


def test_glider_model_params_invalid():
    with pytest.raises(AssertionError):
        GliderModelParams(-1, 0.6, 0.01, 0.7, 1000, 2.0, 1.5, 12.0, 0.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, -1, 0.01, 0.7, 1000, 2.0, 1.5, 12.0, 0.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, 0.6, -1, 0.7, 1000, 2.0, 1.5, 12.0, 0.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, 0.6, 0.01, -0.1, 1000, 2.0, 1.5, 12.0, 0.5)


def test_glider_kinematic_model_control():
    control = GliderKinematicModelControl(0.2, 25)
    assert control.phi == 0.2
    assert control.V == 25


def test_glider_kinematic_model_disturbance():
    disturbance = GliderKinematicModelDisturbance(
        thermal_uplift=1.2, wind_x=2.0, wind_y=-1.0
    )
    assert disturbance.w == 1.2
    assert disturbance.W_x == 2.0
    assert disturbance.W_y == -1.0


def test_open_loop_kinematic_model_step():
    params = DEFAULT_GLIDER_PARAMS
    model = GliderOpenLoopKinematicModel(params)
    control = GliderKinematicModelControl(0.1, 32)
    disturbance = GliderKinematicModelDisturbance(
        thermal_uplift=2.0, wind_x=1.0, wind_y=0.5
    )
    x0, y0, h0, psi0 = model.x, model.y, model.h, model.psi
    model.step(0.5, control, disturbance)
    assert model.x != x0
    assert model.y != y0
    assert model.h != h0
    assert model.psi != psi0
