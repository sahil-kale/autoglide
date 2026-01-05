from dataclasses import dataclass
from enum import Enum, auto
import click
import numpy as np


class SingleAxisPerturberType(Enum):
    """Enumeration of the different types of single axis perturbations."""

    STEP = auto()
    RANDOM = auto()
    CHIRP = auto()


@dataclass
class SingleAxesPerturberStepEvent:
    duration_s: float
    magnitude: float


@dataclass
class SingleAxisPerturberRandomEvent:
    duration_s: float
    magnitude: float


@dataclass
class SingleAxisPerturberChirpEvent:
    duration_s: float
    magnitude: float
    frequency_map: tuple[float, float]  # (start_freq, end_freq)
    theta: float = 0.0  # Phase offset


@dataclass
class SingleAxisPerturberEvent:
    event_type: SingleAxisPerturberType
    event_data: (
        SingleAxesPerturberStepEvent
        | SingleAxisPerturberRandomEvent
        | SingleAxisPerturberChirpEvent
    )


class SingleAxisPerturber:
    def __init__(self, events: list[SingleAxisPerturberEvent]) -> None:
        self.events = events
        self.current_event_index = 0
        self.event_start_time_s = 0.0
        self.active = False

    def start(self, current_time_s: float) -> None:
        self.active = True
        self.current_event_index = 0
        self.event_start_time_s = current_time_s
        self.event_elapsed_time_s = 0.0

    def step(self, current_time_s: float, dt_s: float) -> float:
        if not self.active or self.current_event_index >= len(self.events):
            raise RuntimeError(
                click.secho(
                    "Perturber is not active or all events have been completed.",
                    fg="red",
                    bold=True,
                    err=True,
                )
            )

        current_event = self.events[self.current_event_index]
        elapsed_time_s = current_time_s - self.event_start_time_s

        if elapsed_time_s >= current_event.event_data.duration_s:
            # Move to the next event
            self.current_event_index += 1
            if self.current_event_index < len(self.events):
                self.event_start_time_s = current_time_s
                self.event_elapsed_time_s = 0.0
                current_event = self.events[self.current_event_index]
            else:
                self.active = False  # All events completed
                return 0.0
        else:
            self.event_elapsed_time_s = elapsed_time_s

        return self.get_perturbation(current_event, dt_s)

    def get_perturbation(
        self, current_event: SingleAxisPerturberEvent, dt_s: float
    ) -> float:
        # Apply perturbation based on event type
        if current_event.event_type == SingleAxisPerturberType.STEP:
            return current_event.event_data.magnitude

        if current_event.event_type == SingleAxisPerturberType.RANDOM:
            assert (
                current_event.event_data.magnitude >= 0.0
            ), "Random perturbation magnitude must be non-negative."
            return np.random.uniform(
                -current_event.event_data.magnitude, current_event.event_data.magnitude
            )

        if current_event.event_type == SingleAxisPerturberType.CHIRP:
            if current_event.event_data.frequency_map is None:
                raise RuntimeError(
                    "Chirp event requires a frequency_map (start_freq, end_freq)."
                )
            start_freq, end_freq = current_event.event_data.frequency_map
            t = self.event_elapsed_time_s
            T = current_event.event_data.duration_s
            freq_idx = t / T if T > 0 else 0.0  # Normalized time index
            instantaneous_freq = start_freq + (end_freq - start_freq) * freq_idx
            current_event.event_data.theta += 2 * np.pi * instantaneous_freq * dt_s
            return current_event.event_data.magnitude * np.sin(
                current_event.event_data.theta
            )

        msg = click.secho(
            f"Unknown perturber event type: {current_event.event_type}",
            fg="red",
            bold=True,
            err=True,
        )
        raise RuntimeError(msg)

    def is_active(self) -> bool:
        return self.active
