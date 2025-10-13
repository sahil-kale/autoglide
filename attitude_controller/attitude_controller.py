import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_interface import vehicle_interface
from dataclasses import dataclass


@dataclass
class AttitudeControllerInputs:
    roll_ref_rad: float = 0.0  # desired roll angle, +ve = roll right
    pitch_ref_rad: float = 0.0  # desired pitch angle, +ve = pitch up


class AttitudeController:
    def __init__(self):
        # todo: set up gains?
        pass
