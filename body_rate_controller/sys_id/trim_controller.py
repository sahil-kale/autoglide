import numpy as np
from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleInitialCond,
    JSBSimVehicleConfig,
    JSBSimSimParams,
)
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

        control_output = (
            self.kp * error + self.ki * self.integral_error + self.kd * derivative_error
        )
        return control_output


class GliderAttitudeTrimController:
    def __init__(
        self,
        sim: JSBSim_Sandbox,
        dt: float,
        roll_gains: PIDGains,
        pitch_gains: PIDGains,
        yaw_gains: PIDGains,
    ):
        self.sim = sim
        self.dt = dt
        self.roll_controller = PIDController(roll_gains, dt)
        self.pitch_controller = PIDController(pitch_gains, dt)
        self.sideslip_controller = PIDController(yaw_gains, dt)
        self.sim_truth_state_history = []

    def step_trim_to_attitude(
        self,
        target_roll_rad: float,
        target_pitch_rad: float,
        target_sideslip_rad: float,
        truth_data: SimTruthState,
    ) -> ControlCommands:
        euler_angles = truth_data.attitude.get_euler()
        current_roll_rad = euler_angles[0]
        current_pitch_rad = euler_angles[1]
        current_sideslip_rad = truth_data.sideslip_rad

        aileron_cmd = self.roll_controller.step(target_roll_rad, current_roll_rad)
        elevator_cmd = -self.pitch_controller.step(target_pitch_rad, current_pitch_rad)
        rudder_cmd = self.sideslip_controller.step(
            target_sideslip_rad, current_sideslip_rad
        )

        self.sim_truth_state_history.append(truth_data)

        return ControlCommands(
            aileron_deflection_norm=aileron_cmd,
            elevator_deflection_norm=elevator_cmd,
            rudder_deflection_norm=rudder_cmd,
            spoiler_deflection=0.0,
        )


if __name__ == "__main__":
    # Example usage
    initial_cond = JSBSimVehicleInitialCond(
        h0_m=1000.0,
        vt0_mps=30.0,
        lat0_deg=0.0,
        lon0_deg=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
        psi0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=0.005)
    vehicle_config = JSBSimVehicleConfig(
        model_name="ask21",
        root_dir="jsbsim_sandbox/",
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)

    roll_gains = PIDGains(kp=10, ki=0.1, kd=0.00)
    pitch_gains = PIDGains(kp=10, ki=0.1, kd=0.00)
    yaw_gains = PIDGains(kp=10, ki=0.1, kd=0.00)

    trim_controller = GliderAttitudeTrimController(
        sim,
        dt=0.01,
        roll_gains=roll_gains,
        pitch_gains=pitch_gains,
        yaw_gains=yaw_gains,
    )

    control_commands = ControlCommands(0.0, 0.0, 0.0, 0.0)
    for _step in range(4000):
        truth_data = sim.step(control_commands)
        control_commands = trim_controller.step_trim_to_attitude(
            target_roll_rad=np.deg2rad(45),
            target_pitch_rad=np.deg2rad(-10),
            target_sideslip_rad=np.deg2rad(0),
            truth_data=truth_data,
        )

        # print the above with only 2 decimal places
        print(
            f"Step {_step}: Roll: {sim.fdm.get_property_value('attitude/phi-rad'):.2f}, Pitch: {sim.fdm.get_property_value('attitude/theta-rad'):.2f}, Sideslip: {sim.fdm.get_property_value('aero/beta-rad'):.2f}, Aileron: {control_commands.aileron_deflection_norm:.2f}, Elevator: {control_commands.elevator_deflection_norm:.2f}, Rudder: {control_commands.rudder_deflection_norm:.2f}"
        )

    from jsbsim_sandbox.vehicle_state_visualizer import animate_sim

    # breakpoint()
    animate_sim(
        trim_controller.sim_truth_state_history, interval_ms=0.005, frame_step=10
    )
