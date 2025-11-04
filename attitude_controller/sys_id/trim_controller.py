import numpy as np
from jsbsim_sandbox.sandbox_sim import JSBSim_Sandbox, JSBSimVehicleInitialCond, JSBSimVehicleConfig, JSBSimSimParams
from vehicle_interface.vehicle_interface import *

class PIDGains:
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd

class PIDController:
    def __init__(self, gains: PIDGains, dt: float):
        self.kp = gains.kp
        self.ki = gains.ki
        self.kd = gains.kd
        self.dt = dt
        self.integral_error = 0.0
        self.prev_error = 0.0

    def step(self, target: float, current: float) -> float:
        error = target - current
        self.integral_error += error * self.dt
        derivative_error = (error - self.prev_error) / self.dt
        self.prev_error = error

        control_output = (self.kp * error +
                          self.ki * self.integral_error +
                          self.kd * derivative_error)
        return control_output
    
class AttitudeTrimController:
    def __init__(self, sim: JSBSim_Sandbox, dt: float,
                 roll_gains: PIDGains, pitch_gains: PIDGains, yaw_gains: PIDGains):
        self.sim = sim
        self.dt = dt
        self.roll_controller = PIDController(roll_gains, dt)
        self.pitch_controller = PIDController(pitch_gains, dt)
        self.sideslip_controller = PIDController(yaw_gains, dt)

    def step_trim_to_attitude(self, target_roll_rad: float, target_pitch_rad: float, target_sideslip_rad: float, sensor_data: MockSensors) -> ControlCommands:
        # for now, get all values we need directly from the sim.
        current_roll_rad = self.sim.fdm.get_property_value("attitude/phi-rad")
        current_pitch_rad = self.sim.fdm.get_property_value("attitude/theta-rad")
        current_sideslip_rad = self.sim.fdm.get_property_value("aero/beta-rad")

        aileron_cmd = self.roll_controller.step(target_roll_rad, current_roll_rad)
        elevator_cmd = self.pitch_controller.step(target_pitch_rad, current_pitch_rad)
        rudder_cmd = self.sideslip_controller.step(target_sideslip_rad, current_sideslip_rad)

        return ControlCommands(aileron=aileron_cmd, elevator=elevator_cmd, rudder=rudder_cmd)
