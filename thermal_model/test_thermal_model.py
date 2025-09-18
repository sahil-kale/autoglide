import numpy as np
import pytest
from thermal_model.thermal_model import ThermalModel, ThermalModelParams


def test_thermal_model_init():
    params = ThermalModelParams(5.0, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
    model = ThermalModel(params=params)
    assert model.w_max == 5.0
    assert model.r_th == 50.0
    assert model.x_c == 0.0
    assert model.y_c == 0.0
    assert model.V_e == 1.0
    assert model.kx == 0.03
    assert model.ky == -0.02
    assert model.core_center_random_noise_std == 1.5


def test_core_center_at_height():
    params = ThermalModelParams(5.0, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 0.0)
    model = ThermalModel(params=params)
    x_c, y_c = model.core_center_at_height(100)
    assert np.isclose(x_c, 3.0)
    assert np.isclose(y_c, -2.0)


def test_get_thermal_uplift_center():
    params = ThermalModelParams(5.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    model = ThermalModel(params=params)
    w = model.get_thermal_uplift(0.0, 0.0, 0.0)
    assert np.isclose(w, 5.0)


def test_get_thermal_uplift_off_center():
    params = ThermalModelParams(5.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    model = ThermalModel(params=params)
    w = model.get_thermal_uplift(50.0, 0.0, 0.0)
    expected = (5.0 + 1.0) * np.exp(-1.0) - 1.0
    assert np.isclose(w, expected)


def test_invalid_params():
    with pytest.raises(AssertionError):
        params = ThermalModelParams(-1, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
        ThermalModel(params=params)
    with pytest.raises(AssertionError):
        params = ThermalModelParams(5.0, 0.5, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
        ThermalModel(params=params)
    with pytest.raises(AssertionError):
        params = ThermalModelParams(5.0, 50.0, 0.0, 0.0, -1.0, 0.03, -0.02, 1.5)
        ThermalModel(params=params)
