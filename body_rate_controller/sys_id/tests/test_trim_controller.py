from body_rate_controller.sys_id.trim_controller import (
    GliderAttitudeTrimController,
    TrimConfig,
)
import numpy as np
from jsbsim_sandbox.vehicle_state_visualizer import animate_sim
from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
)
from utils.pid import PIDConfig


def sanity_test_trim_controller():
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

    roll_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)
    pitch_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)
    yaw_gains = PIDConfig(kp=10, ki=0.1, kd=0.00)

    trim_controller = GliderAttitudeTrimController(
        sim,
        dt=0.01,
        roll_gains=roll_gains,
        pitch_gains=pitch_gains,
        yaw_gains=yaw_gains,
        trim_config=TrimConfig(
            trim_angle_threshold_deg=5.0,
            max_d_body_rate_degps2=0.1,
        ),
    )

    assert trim_controller.run_until_trim(
        target_roll_rad=np.deg2rad(60),
        target_pitch_rad=np.deg2rad(0),
        target_sideslip_rad=np.deg2rad(0),
    )

    # Ensure that the final state is within the trim thresholds
    final_state = trim_controller.sim.get_sim_truth_state()
    roll, pitch, yaw = final_state.attitude.to_euler_angles()
    assert abs(roll - np.deg2rad(60)) <= np.deg2rad(5.0)
    assert abs(pitch - np.deg2rad(0)) <= np.deg2rad(5.0)
    assert abs(final_state.sideslip_rad - np.deg2rad(0)) <= np.deg2rad(5.0)
