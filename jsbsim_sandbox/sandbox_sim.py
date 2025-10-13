import jsbsim
import numpy as np
import argparse
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils import units
from dataclasses import dataclass
from vehicle_interface.vehicle_interface import *


@dataclass
class JSBSimVehicleInitialCond:
    h0_m: float = 800.0
    vt0_mps: float = 40.0
    lat0_deg: float = 37.4275
    lon0_deg: float = -122.1697
    psi0_deg: float = 90.0


@dataclass
class JSBSimVehicleConfig:
    model_name: str
    aileron_multiplier: float
    elevator_multiplier: float
    rudder_multiplier: float
    spoiler_max_deflection: float  # [0, 1], fraction of full def

    root_dir: str = None  # JSBSim root directory, or None to use default search paths


@dataclass
class JSBSimSimParams:
    dt_s: float = 0.01


class JSBSim_Sandbox:
    def __init__(
        self,
        initial_cond: JSBSimVehicleInitialCond,
        sim_params: JSBSimSimParams,
        vehicle_config: JSBSimVehicleConfig,
    ):
        self.fdm = jsbsim.FGFDMExec(root_dir=vehicle_config.root_dir)
        self.fdm.set_debug_level(1)
        self.fdm.set_dt(sim_params.dt_s)

        if not self.fdm.load_model(vehicle_config.model_name):
            raise RuntimeError(
                f"Failed to load aircraft model '{vehicle_config.model_name}'"
            )

        self.fdm["ic/h-sl-ft"] = units.meters_to_feet(initial_cond.h0_m)
        self.fdm["ic/vt-kts"] = units.mps_to_knots(initial_cond.vt0_mps)
        self.fdm["ic/lat-gc-deg"] = initial_cond.lat0_deg
        self.fdm["ic/long-gc-deg"] = initial_cond.lon0_deg
        self.fdm["ic/psi-true-deg"] = initial_cond.psi0_deg

        self.fdm.run_ic()

        self.sim_params = sim_params
        self.initial_cond = initial_cond
        self.vehicle_config = vehicle_config

    def _normalize_and_clip_axis(self, value: float, multiplier: float) -> float:
        norm_value = value * multiplier
        return np.clip(norm_value, -1.0, 1.0)

    def step(self, control_commands: ControlCommands) -> MockSensors:
        aileron_cmd_norm = self._normalize_and_clip_axis(
            control_commands.aileron_deflection_norm,
            self.vehicle_config.aileron_multiplier,
        )
        elevator_cmd_norm = self._normalize_and_clip_axis(
            control_commands.elevator_deflection_norm,
            self.vehicle_config.elevator_multiplier,
        )
        rudder_cmd_norm = self._normalize_and_clip_axis(
            control_commands.rudder_deflection_norm,
            self.vehicle_config.rudder_multiplier,
        )
        spoiler_cmd_norm = np.clip(control_commands.spoiler_deflection, 0, 1)

        self.fdm["fcs/aileron-cmd-norm"] = aileron_cmd_norm
        self.fdm["fcs/elevator-cmd-norm"] = elevator_cmd_norm
        self.fdm["fcs/rudder-cmd-norm"] = rudder_cmd_norm
        self.fdm["fcs/speedbrake-cmd-norm"] = spoiler_cmd_norm
        self.fdm.run()

        phi = self.fdm.get_property_value("attitude/phi-rad")  # roll
        theta = self.fdm.get_property_value("attitude/theta-rad")  # pitch
        psi = self.fdm.get_property_value("attitude/psi-rad")  # yaw

        sensors = MockSensors(
            airspeed_mps=units.knots_to_mps(self.fdm["velocities/vtrue-kts"]),
            altitude_m=units.feet_to_meters(self.fdm["position/h-sl-ft"]),
            latitude_deg=self.fdm["position/lat-gc-deg"],
            longitude_deg=self.fdm["position/long-gc-deg"],
            attitude=Quaternion.from_euler(
                roll=phi, pitch=theta, yaw=psi
            ).normalize(),  # TODO: Consider just being dependent on IMU measurements here instead. But we'll assume the attitude is already known since that problem is solved
            p_radps=self.fdm["velocities/p-rad_sec"],
            q_radps=self.fdm["velocities/q-rad_sec"],
            r_radps=self.fdm["velocities/r-rad_sec"],
        )
        return sensors


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="JSBSim Single Landing Simulation Demo"
    )
    parser.add_argument(
        "--aircraft",
        type=str,
        required=True,
        help="Aircraft model name (e.g., 'glider')",
    )
    parser.add_argument(
        "--h0_m", type=float, default=800.0, help="Initial altitude in meters"
    )
    parser.add_argument(
        "--vt0_mps", type=float, default=40.0, help="Initial true airspeed in m/s"
    )
    parser.add_argument(
        "--lat0_deg", type=float, default=37.4275, help="Initial latitude in degrees"
    )
    parser.add_argument(
        "--lon0_deg", type=float, default=-122.1697, help="Initial longitude in degrees"
    )
    parser.add_argument(
        "--psi0_deg", type=float, default=90.0, help="Initial heading in degrees"
    )
    parser.add_argument(
        "--dt", type=float, default=0.01, help="Simulation time step in seconds"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="Total simulation duration in seconds",
    )
    parser.add_argument(
        "--jsbsim_root", type=str, default=None, help="JSBSim root directory (optional)"
    )
    args = parser.parse_args()

    initial_cond = JSBSimVehicleInitialCond(
        h0_m=args.h0_m,
        vt0_mps=args.vt0_mps,
        lat0_deg=args.lat0_deg,
        lon0_deg=args.lon0_deg,
        psi0_deg=args.psi0_deg,
    )
    sim_params = JSBSimSimParams(dt_s=args.dt)
    vehicle_config = JSBSimVehicleConfig(
        model_name=args.aircraft,
        root_dir=args.jsbsim_root,
        aileron_multiplier=1.0,
        elevator_multiplier=1.0,
        rudder_multiplier=1.0,
        spoiler_max_deflection=1.0,
    )

    sim = JSBSim_Sandbox(initial_cond, sim_params, vehicle_config)
    vehicle_interface = VehicleInterface()
    vehicle_interface.control_commands.elevator_deflection_norm = 0.0

    num_steps = int(args.duration / args.dt)
    for step in range(num_steps):
        commands = vehicle_interface.control_commands
        sensors = sim.step(commands)
        vehicle_interface.sensors = sensors

        if step % 100 == 0:
            print(
                f"Time: {step * args.dt:.2f}s, Altitude: {sensors.altitude_m:.1f}m, Airspeed: {sensors.airspeed_mps:.1f}m/s"
            )
