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

    def __str__(self):
        return (
            f"Aileron Deflection (norm): {self.aileron_deflection_norm:.4f}, "
            f"Elevator Deflection (norm): {self.elevator_deflection_norm:.4f}, "
            f"Rudder Deflection (norm): {self.rudder_deflection_norm:.4f}, "
            f"Spoiler Deflection: {self.spoiler_deflection:.4f}"
        )


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

    control_commands: ControlCommands = field(
        default_factory=lambda: ControlCommands(0.0, 0.0, 0.0, 0.0)
    )

    aoa_rad: float = 0.0  # angle of attack, +ve = nose up (alpha > 0)

    def __str__(self):
        return (
            f"Time: {self.time_s:.2f} s, \n"
            f"TAS: {self.airspeed_mps:.2f} m/s, \n"
            f"Alt: {self.altitude_m:.2f} m, \n"
            f"Lat: {self.latitude_deg:.6f} deg, \n"
            f"Lon: {self.longitude_deg:.6f} deg, \n"
            f"Attitude (quat): ({self.attitude.w:.4f}, {self.attitude.x:.4f}, {self.attitude.y:.4f}, {self.attitude.z:.4f}), \n"
            f"Sideslip: {np.degrees(self.sideslip_rad):.2f} deg, \n"
            f"p: {np.degrees(self.p_radps):.2f} deg/s, \n"
            f"q: {np.degrees(self.q_radps):.2f} deg/s, \n"
            f"r: {np.degrees(self.r_radps):.2f} deg/s\n"
            f"Control Commands: {self.control_commands}\n"
        )
