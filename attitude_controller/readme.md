# Inner Loop Controller
The goal of the inner loop controller is to track desired pitch and roll angles by commanding appropriate deflection of the aircraft's control surfaces (ailerons, elevator, rudder).

Note that this module assumes the end goal is coordinated flight - as a result, the rudder angle is controlled to maintain zero sideslip (beta = 0).

## Convention
Variable names:

### Inertial Frame References (flat-earth NED):
- $\phi$ : roll angle (rad)
- $\theta$ : pitch angle (rad)

### Body Frame:
See [here](../vehicle_interface/readme.md#convention) for a description of the aerospace convention used in this module.
#### Control Surfaces:
- $\delta_{aileron}$: Aileron deflection (normalized -1 to 1, where positive deflects right aileron up, inducing roll to the right)
- $\delta_{elevator}$: Elevator deflection (normalized -1 to 1, where positive deflects elevator up, inducing pitch up)
- $\delta_{rudder}$: Rudder deflection (normalized -1 to 1, where positive deflects rudder to the right, inducing yaw to the right)

#### State Variables:
- $\omega_{p}$: Body angular rate about the x-axis (roll rate - rad/s)
- $\omega_{q}$: Body angular rate about the y-axis (pitch rate - rad/s)
- $\omega_{r}$: Body angular rate about the z-axis (yaw rate - rad/s)

## System Identification
In order to determine the inner loop controller structure (initally) and gains (ultimately), a system identification procedure is required. This involves 
- Attempting to find stable-trim conditions for the aircraft at various pitch and roll angles.
- Derive a small-signal linear model of the aircraft at these trim conditions (think A, B, C, D matrices).

## Inputs From Outer Loop
Note that what is described as the "outer loop" here is the lateral-directional and longitudinal controllers that generate attitude commands for the inner loop controller.
- Desired roll angle (phi_cmd) in radians
- Desired pitch angle (theta_cmd) in radians

## Todo
- Implement sink-rate control (either in here or a separate module)