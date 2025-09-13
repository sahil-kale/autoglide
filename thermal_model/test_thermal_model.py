import numpy as np
import pytest
from thermal_model.thermal_model import ThermalModel


def test_thermal_model_init():
    model = ThermalModel(5.0, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
    assert model.w_max == 5.0
    assert model.r_th == 50.0
    assert model.x_th == 0.0
    assert model.y_th == 0.0
    assert model.V_e == 1.0
    assert model.kx == 0.03
    assert model.ky == -0.02
    assert model.core_center_random_noise_std == 1.5


def test_core_center_at_height():
    model = ThermalModel(5.0, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 0.0)
    x_c, y_c = model.core_center_at_height(100)
    assert np.isclose(x_c, 3.0)
    assert np.isclose(y_c, -2.0)


def test_get_thermal_uplift_center():
    model = ThermalModel(5.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    w = model.get_thermal_uplift(0.0, 0.0, 0.0)
    assert np.isclose(w, 5.0)


def test_get_thermal_uplift_off_center():
    model = ThermalModel(5.0, 50.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    w = model.get_thermal_uplift(50.0, 0.0, 0.0)
    expected = (5.0 + 1.0) * np.exp(-1.0) - 1.0
    assert np.isclose(w, expected)


def test_invalid_params():
    with pytest.raises(AssertionError):
        ThermalModel(-1, 50.0, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
    with pytest.raises(AssertionError):
        ThermalModel(5.0, 0.5, 0.0, 0.0, 1.0, 0.03, -0.02, 1.5)
    with pytest.raises(AssertionError):
        ThermalModel(5.0, 50.0, 0.0, 0.0, -1.0, 0.03, -0.02, 1.5)
