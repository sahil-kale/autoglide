import numpy as np
from dataclasses import dataclass, field
from utils.quaternion import Quaternion


@dataclass
class ControlCommands:
    aileron_deflection_norm: (
        float  # +ve = right aileron down / roll right (right wing down, p > 0)
    )
    elevator_deflection_norm: (
        float  # +ve = elevator trailing edge down / (pitch down, q < 0)
    )
    rudder_deflection_norm: float  # +ve = trailing edge left / yaw left (r > 0)
    spoiler_deflection: float  # ≥ 0 = spoilers deployed (symmetric lift dump)


@dataclass
class SimTruthState:
    time_s: float = 0.0  # simulation time
    airspeed_mps: float = 0.0  # true airspeed
    altitude_m: float = 0.0  # altitude above sea level
    latitude_deg: float = 0.0  # degrees
    longitude_deg: float = 0.0  # degrees
    attitude: Quaternion = field(
        default_factory=lambda: Quaternion(1, 0, 0, 0)
    )  # unit quaternion (w, x, y, z)
    sideslip_rad: float = 0.0  # +ve = nose left / yaw left (beta > 0)
    p_radps: float = 0.0  # roll rate, +ve = right wing down / roll right (p > 0)
    q_radps: float = 0.0  # pitch rate, +ve = pitch up (q > 0)
    r_radps: float = 0.0  # yaw rate, +ve = nose left / yaw left (r > 0)
