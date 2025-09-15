import numpy as np
import pytest
from thermal_estimator.thermal_estimator import ThermalEstimator
from utils.location import WorldFrameCoordinate


def test_thermal_estimator_initialization():
    est = ThermalEstimator(num_samples_to_buffer=10)
    assert est.num_samples_to_buffer == 10
    assert isinstance(est.samples, list)
    assert len(est.samples) == 0


def test_thermal_model_estimate_updraft():
    est = ThermalEstimator(10)
    aircraft = WorldFrameCoordinate(10, 0)
    thermal = WorldFrameCoordinate(0, 0)
    w = est.thermal_model_estimate_updraft(aircraft, thermal, W0=5, Rth=50)
    assert np.isclose(w, 5 * np.exp(-0.04), atol=1e-2)


def test_step_and_estimation():
    est = ThermalEstimator(5)
    for i in range(6):
        loc = WorldFrameCoordinate(i, 0)
        meas = 5 * np.exp(-((i / 50) ** 2))
        est.step(meas, loc)
    params = est.estimated_params
    assert len(params) == 4
    assert params[0] > 0
    assert params[1] > 0


def test_confidence_update():
    est = ThermalEstimator(5)
    for i in range(6):
        loc = WorldFrameCoordinate(i, 0)
        meas = 5 * np.exp(-((i / 50) ** 2))
        est.step(meas, loc)
    conf = est.get_confidence()
    assert 0.0 <= conf <= 1.0
