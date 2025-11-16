from dataclasses import dataclass
from enum import Enum, auto
import click
import numpy as np


class SingleAxisPerturberType(Enum):
    """Enumeration of the different types of single axis perturbations."""

    STEP = auto()
    RANDOM = auto()


@dataclass
class SingleAxisPerturberEvent:
    event_type: SingleAxisPerturberType
    magnitude: (
        float  # perturbation magnitude (for step: step size, for random: max amplitude)
    )
    duration_s: float


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

    def step(self, current_time_s: float) -> float:
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

        if elapsed_time_s >= current_event.duration_s:
            # Move to the next event
            self.current_event_index += 1
            if self.current_event_index < len(self.events):
                self.event_start_time_s = current_time_s
                current_event = self.events[self.current_event_index]
            else:
                self.active = False  # All events completed
                return 0.0

        # Apply perturbation based on event type
        if current_event.event_type == SingleAxisPerturberType.STEP:
            return current_event.magnitude
        elif current_event.event_type == SingleAxisPerturberType.RANDOM:
            assert (
                current_event.magnitude >= 0.0
            ), "Random perturbation magnitude must be non-negative."
            return np.random.uniform(-current_event.magnitude, current_event.magnitude)

        msg = click.secho(
            f"Unknown perturber event type: {current_event.event_type}",
            fg="red",
            bold=True,
            err=True,
        )
        raise RuntimeError(msg)

    def is_active(self) -> bool:
        return self.active
