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
from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt

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
    
@dataclass
class SingleAxisModelIdentificationData:
    time_s: np.ndarray[float]
    input_deflection_norm: np.ndarray[float]
    output_response_radps: np.ndarray[float]
    
    def plot(self, title: str = "Single Axis Model Identification Data"):
        # Plot the input and output data
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
        fig.suptitle(title)
        ax1.plot(self.time_s, self.input_deflection_norm, label="Input Deflection Norm")
        ax1.set_ylabel("Deflection Norm")
        ax1.legend()
        
        ax2.plot(self.time_s, self.output_response_radps, label="Output Response (rad/s)", color='orange')
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Output Response (rad/s)")
        ax2.legend()
        
        plt.tight_layout()
        plt.show()

    
    
def get_bode(data: SingleAxisModelIdentificationData, dt_s: float, nperseg: int = 4096):
    """
    Estimate frequency response G(jw) from input u and output y using spectral estimation:
        G = S_yu / S_uu

    Returns:
        freq_hz, mag_db, phase_deg, coh
    """
    import numpy as np
    from scipy.signal import csd, welch, coherence, detrend

    u = np.asarray(data.input_deflection_norm)
    y = np.asarray(data.output_response_radps)

    # basic hygiene
    u = detrend(u, type="constant")
    y = detrend(y, type="constant")

    fs = 1.0 / dt_s

    # Use same windowing/segmenting for all estimates
    f, Puu = welch(u, fs=fs, nperseg=min(nperseg, len(u)), window="hann", detrend=False)
    _, Pyu = csd(y, u, fs=fs, nperseg=min(nperseg, len(u)), window="hann", detrend=False)
    coh = coherence(y, u, fs=fs, nperseg=min(nperseg, len(u)), window="hann", detrend=False)

    # FRF estimate
    G = Pyu / Puu

    mag_db = 20.0 * np.log10(np.maximum(np.abs(G), 1e-12))
    phase_deg = np.angle(G, deg=True)

    return f, mag_db, phase_deg, coh

def plot_bode_with_coherence(freq_hz, mag_db, phase_deg, coh, title="Bode"):
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 8))

    plt.subplot(2, 1, 1)
    plt.semilogx(freq_hz, mag_db)
    plt.title(title)
    plt.ylabel("Magnitude (dB)")
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)

    plt.subplot(2, 1, 2)
    plt.semilogx(freq_hz, phase_deg)
    plt.ylabel("Phase (deg)")
    plt.grid(True, which="both", linestyle="--", linewidth=0.5)

    # plt.subplot(3, 1, 3)
    # plt.semilogx(freq_hz, coh)
    # plt.ylim([0, 1.05])
    # plt.ylabel("Coherence")
    # plt.xlabel("Frequency (Hz)")
    # plt.grid(True, which="both", linestyle="--", linewidth=0.5)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    elevator_perturbation_truth_data = perturb_elevator()
    identification_data = SingleAxisModelIdentificationData(
        time_s=np.array([
            data_point.time_s for data_point in elevator_perturbation_truth_data
        ]),
        input_deflection_norm=np.array([
            data_point.control_commands.elevator_deflection_norm
            for data_point in elevator_perturbation_truth_data
        ]),
        output_response_radps=np.array([
            data_point.q_radps
            for data_point in elevator_perturbation_truth_data
        ]),
    )
    identification_data.plot(
        title="Elevator Perturbation - Raw Data"
    )
    
    print(f"Generating bode plot from {len(identification_data.time_s)} data points.")
    
    freq_hz, mag, phase, coh = get_bode(
        identification_data,
        dt_s=0.01,
        nperseg=4096,
    )
    plot_bode_with_coherence(freq_hz, mag, phase, coh, title="Elevator Perturbation - Bode Plot with Coherence")
    