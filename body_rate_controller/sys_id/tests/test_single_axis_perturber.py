import pytest
import numpy as np

from body_rate_controller.sys_id.single_axis_perturber import (
    SingleAxisPerturberType,
    SingleAxisPerturberEvent,
    SingleAxisPerturber,
)


def test_step_raises_if_not_active():
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, 1.0, 1.0)]
    perturber = SingleAxisPerturber(events)

    with pytest.raises(RuntimeError):
        perturber.step(current_time_s=0.0)

    assert perturber.is_active() is False


def test_start_initializes_state():
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, 1.0, 1.0)]
    perturber = SingleAxisPerturber(events)

    perturber.start(current_time_s=10.0)

    assert perturber.is_active() is True
    assert perturber.current_event_index == 0
    assert perturber.event_start_time_s == 10.0


def test_step_event_constant_output_for_step_type():
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, 2.5, 1.0)]
    perturber = SingleAxisPerturber(events)

    perturber.start(current_time_s=0.0)

    # During the event duration we should get constant magnitude
    assert perturber.step(current_time_s=0.0) == pytest.approx(2.5)
    assert perturber.step(current_time_s=0.5) == pytest.approx(2.5)
    assert perturber.step(current_time_s=0.99) == pytest.approx(2.5)


def test_step_event_becomes_inactive_after_duration():
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, 1.0, 1.0)]
    perturber = SingleAxisPerturber(events)

    perturber.start(current_time_s=0.0)

    # Still active before duration is reached
    _ = perturber.step(current_time_s=0.5)
    assert perturber.is_active() is True

    # Once elapsed_time_s >= duration_s and this is the last event, it should deactivate
    output = perturber.step(current_time_s=1.5)
    assert output == pytest.approx(0.0)
    assert perturber.is_active() is False

    # Further calls should raise
    with pytest.raises(RuntimeError):
        perturber.step(current_time_s=2.0)


def test_multiple_step_events_progression():
    events = [
        SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, 1.0, 1.0),
        SingleAxisPerturberEvent(SingleAxisPerturberType.STEP, -0.5, 2.0),
    ]
    perturber = SingleAxisPerturber(events)
    perturber.start(current_time_s=0.0)

    # First event
    assert perturber.step(current_time_s=0.2) == pytest.approx(1.0)
    assert perturber.current_event_index == 0

    # Let time go well beyond the first duration, then call again.
    # This call will advance to the next event in the internal state.
    _ = perturber.step(current_time_s=1.2)
    # After that call, it should now have moved to the next event
    assert perturber.current_event_index == 1
    assert perturber.is_active() is True

    # Now we should see the second event's magnitude on subsequent steps
    assert perturber.step(current_time_s=1.3) == pytest.approx(-0.5)
    assert perturber.step(current_time_s=2.9) == pytest.approx(-0.5)

    # After the second event has expired, it should deactivate
    output = perturber.step(current_time_s=3.5)
    assert output == pytest.approx(0.0)
    assert perturber.is_active() is False


def test_random_event_uses_uniform_distribution(monkeypatch):
    # Make the RNG deterministic for this test
    calls = {}

    def fake_uniform(low, high):
        # basic sanity checks
        calls["low"] = low
        calls["high"] = high
        # return midpoint deterministically
        return (low + high) / 2.0

    monkeypatch.setattr(np.random, "uniform", fake_uniform)

    magnitude = 3.0
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.RANDOM, magnitude, 1.0)]
    perturber = SingleAxisPerturber(events)
    perturber.start(current_time_s=0.0)

    value = perturber.step(current_time_s=0.1)

    # Check that np.random.uniform was called with correct bounds
    assert calls["low"] == -magnitude
    assert calls["high"] == magnitude

    # And that the returned value is within the expected range
    assert -magnitude <= value <= magnitude
    # For our fake_uniform, it's the midpoint
    assert value == pytest.approx(0.0)


def test_random_event_negative_magnitude_asserts():
    events = [SingleAxisPerturberEvent(SingleAxisPerturberType.RANDOM, -1.0, 1.0)]
    perturber = SingleAxisPerturber(events)
    perturber.start(current_time_s=0.0)

    with pytest.raises(AssertionError):
        perturber.step(current_time_s=0.1)
