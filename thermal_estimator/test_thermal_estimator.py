import numpy as np
import pytest
from thermal_estimator.thermal_estimator import (
    ThermalEstimator,
    ReducedOrderGaussianThermalModel,
    ThermalEstimate,
)
from utils.location import WorldFrameCoordinate


def test_thermal_estimator_initialization():
    est = ThermalEstimator(num_samples_to_buffer=10)
    assert est.num_samples_to_buffer == 10
    assert isinstance(est.samples, list)
    assert len(est.samples) == 0


def test_thermal_model_estimate_updraft():
    estimate = ThermalEstimate(
        W0=5, Rth=50, est_core=WorldFrameCoordinate(0, 0), confidence=0.8
    )
    aircraft = WorldFrameCoordinate(10, 0)
    thermal = WorldFrameCoordinate(0, 0)
    gaussian_model = ReducedOrderGaussianThermalModel(thermal_estimate=estimate)

    w = gaussian_model.thermal_model_estimate_updraft(aircraft)
    assert np.isclose(w, 5 * np.exp(-0.04), atol=1e-2)


def test_step_and_estimation():
    est = ThermalEstimator(5)
    for i in range(6):
        loc = WorldFrameCoordinate(i, 0)
        meas = 5 * np.exp(-((i / 50) ** 2))
        est.step(meas, loc)
    estimate = est.get_estimate()
    assert estimate.W0 > 0
    assert estimate.Rth > 0
    assert isinstance(estimate.est_core, WorldFrameCoordinate)
    assert 0 <= estimate.confidence <= 1


def test_confidence_update():
    est = ThermalEstimator(5)
    for i in range(6):
        loc = WorldFrameCoordinate(i, 0)
        meas = 5 * np.exp(-((i / 50) ** 2))
        est.step(meas, loc)
    estimate = est.get_estimate()
    confidence = estimate.confidence
    assert 0 <= confidence <= 1
