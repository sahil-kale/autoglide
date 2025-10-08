import numpy as np
from dataclasses import dataclass

@dataclass
class ControlCommands:
    aileron_deflection_rad: float  # +ve = right aileron down / roll right (right wing down, p > 0)
    elevator_deflection_rad: float # +ve = trailing edge up / pitch up (q > 0)
    rudder_deflection_rad: float   # +ve = trailing edge left / yaw left (r > 0)
    spoiler_deflection: float      # ≥ 0 = spoilers deployed (symmetric lift dump)

@dataclass 
class MockSensors:
    airspeed_mps: float = 0.0 # true airspeed
    altitude_m: float = 0.0   # altitude above sea level
    latitude_deg: float = 0.0  # degrees
    longitude_deg: float = 0.0 # degrees
    roll_rad: float = 0.0      # +ve = right wing down / roll right (p > 0)
    pitch_rad: float = 0.0     # +ve = pitch up (q > 0)
    yaw_rad: float = 0.0       # +ve = nose left / yaw left (r > 0)
    p_radps: float = 0.0        # roll rate, +ve = right wing down / roll right (p > 0)
    q_radps: float = 0.0        # pitch rate, +ve = pitch up (q > 0)
    r_radps: float = 0.0        # yaw rate, +ve = nose left / yaw left (r > 0)

class VehicleInterface:
    def __init__(self):
        self.control_commands = ControlCommands(0.0, 0.0, 0.0, 0.0)
        self.sensors = MockSensors()

    def send_control_commands(self, commands: ControlCommands):
        self.control_commands = commands
    
    def read_sensors(self) -> MockSensors:
        return self.sensors
