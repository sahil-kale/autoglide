# Inner Loop Controller
The goal of the inner loop controller is to track desired pitch and roll angles by commanding appropriate deflection of the aircraft's control surfaces (ailerons, elevator, rudder).

Note that this module assumes the end goal is coordinated flight - as a result, the rudder angle is controlled to maintain zero sideslip (beta = 0).

## Convention
See [here](../vehicle_interface/readme.md#convention) for a description of the aerospace convention used in this module.

## Inputs From Outer Loop
Note that what is described as the "outer loop" here is the lateral-directional and longitudinal controllers that generate attitude commands for the inner loop controller.
- Desired roll angle (phi_cmd) in radians
- Desired pitch angle (theta_cmd) in radians

## Todo
- Implement sink-rate control (either in here or a separate module)