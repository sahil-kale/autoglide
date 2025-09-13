import numpy as np
import pytest
from glider_model.model import (
    GliderModelParams,
    GliderKinematicModelControl,
    GliderKinematicModelDisturbance,
    GliderOpenLoopKinematicModel,
)


def test_glider_model_params_valid():
    params = GliderModelParams(30, 0.6, 0.01, 0.7, 1000, 2.0, 1.5)
    assert params.V_star == 30
    assert params.s_min == 0.6
    assert params.k_v == 0.01
    assert params.alpha_n == 0.7
    assert params.initial_altitude == 1000
    assert params.vel_tau == 1.5
    assert params.roll_tau == 2.0


def test_glider_model_params_invalid():
    with pytest.raises(AssertionError):
        GliderModelParams(-1, 0.6, 0.01, 0.7, 1000, 2.0, 1.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, -1, 0.01, 0.7, 1000, 2.0, 1.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, 0.6, -1, 0.7, 1000, 2.0, 1.5)
    with pytest.raises(AssertionError):
        GliderModelParams(30, 0.6, 0.01, -0.1, 1000, 2.0, 1.5)


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
    params = GliderModelParams(30, 0.6, 0.01, 0.7, 1000, 2.0, 1.5)
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


def test_sink_rate_formula():
    params = GliderModelParams(30, 0.6, 0.01, 0.7, 1000, 2.0, 1.5)
    model = GliderOpenLoopKinematicModel(params)
    V = 30
    phi = 0.2
    s = model.get_sink_rate(V, phi, params)
    assert s > 0
    s_zero_bank = model.get_sink_rate(V, 0.0, params)
    assert s_zero_bank < s
