from jsbsim_sandbox.sandbox_sim import (
    JSBSim_Sandbox,
    JSBSimVehicleConfig,
    JSBSimVehicleInitialCond,
    JSBSimSimParams,
)
from vehicle_interface.vehicle_interface import ControlCommands
from utils.pid import PIDConfig
from body_rate_controller.sys_id.trim_controller import (
    GliderAttitudeTrimController,
    TrimConfig,
    TrimTarget,
)

from body_rate_controller.sys_id.single_axis_perturber import (
    SingleAxisPerturber,
    SingleAxisPerturberChirpEvent,
    SingleAxisPerturberEvent,
    SingleAxisPerturberType,
)

def perturb_elevator():
    # Initialize the simulation environment
    initial_cond = JSBSimVehicleInitialCond(
        h0_m=2000.0,
        vt0_mps=37.0,
        lat0_deg=0.0,
        lon0_deg=0.0,
        phi0_rad=0.0,
        theta0_rad=0.0,
        psi0_rad=0.0,
    )
    sim_params = JSBSimSimParams(dt_s=0.01)
    vehicle_config = JSBSimVehicleConfig(
        model_name="ask21",
        root_dir="jsbsim_sandbox/",
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)

    # Configure the trim controller
    roll_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)
    pitch_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)
    yaw_gains = PIDConfig(kp=10, ki=0.1, kd=0.0)

    trim_controller = GliderAttitudeTrimController(
        sim,
        dt=0.01,
        roll_gains=roll_gains,
        pitch_gains=pitch_gains,
        yaw_gains=yaw_gains,
        trim_config=TrimConfig(
            trim_angle_threshold_deg=1.0,
            max_d_body_rate_degps2=0.1,
            trim_persistence_threshold_s=2.0,
            max_rate_of_change_airspeed_mps2=0.5,
        ),
    )

    # Trim the plane to 0 degrees roll, pitch, and sideslip
    trim_target = TrimTarget(
        roll_rad=0.0,
        pitch_rad=0.0,
        sideslip_rad=0.0,
    )

    trim_controller.run_until_trim(
        trim_target=trim_target,
        max_steps=8000,
        raise_on_fail=True,
    )

    # Define a chirp perturbation event
    chirp_event = SingleAxisPerturberEvent(
        event_type=SingleAxisPerturberType.CHIRP,
        event_data=SingleAxisPerturberChirpEvent(
            duration_s=50.0,
            magnitude=1.0,
            frequency_map=(1.0, 10.0),
        ),
    )

    perturber = SingleAxisPerturber(events=[chirp_event])
    perturber.start(current_time_s=sim.get_sim_time_s())
    
    perturbation_truth_data = []

    # Apply the chirp perturbation
    while sim.get_sim_time_s() < chirp_event.event_data.duration_s:
        perturbation = perturber.step(
            current_time_s=sim.get_sim_time_s(), dt_s=sim_params.dt_s
        )

        control_commands = ControlCommands(0.0, 0.0, 0.0, 0.0)
        # Apply perturbation to the aileron as an example
        control_commands.elevator_deflection_norm = (
            perturbation
            + trim_controller.sim_truth_state_history[
                -1
            ].control_commands.elevator_deflection_norm
        )

        truth_data = sim.step(control_commands)
        perturbation_truth_data.append(truth_data)
        
    # return the truth data from the perturbation time -> time now
    return perturbation_truth_data
    
    
def plot_time_domain_chirp_data(perturbation_truth_data):
    import matplotlib.pyplot as plt

    times = [data.time_s for data in perturbation_truth_data]
    elevator_deflections = [
        data.control_commands.elevator_deflection_norm for data in perturbation_truth_data
    ]
    body_rates_q = [data.q_radps for data in perturbation_truth_data]
    
    assert len(times) == len(elevator_deflections) == len(body_rates_q), f"Lengths do not match: {len(times)}, {len(elevator_deflections)}, {len(body_rates_q)}"
    
    # Overlay plots of elevator deflection and body rate q (different y-axes)
    fig, ax1 = plt.subplots()
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Elevator Deflection (norm)", color="tab:blue")
    ax1.plot(times, elevator_deflections, color="tab:blue", label="Elevator Deflection")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax2 = ax1.twinx()
    ax2.set_ylabel("Body Rate q (rad/s)", color="tab:red")
    ax2.plot(times, body_rates_q, color="tab:red", label="Body Rate q")
    ax2.tick_params(axis="y", labelcolor="tab:red")
    fig.tight_layout()
    plt.title("Elevator Deflection and Body Rate q over Time")
    plt.show()

if __name__ == "__main__":
    elevator_perturbation_truth_data = perturb_elevator()
    plot_time_domain_chirp_data(elevator_perturbation_truth_data)
    