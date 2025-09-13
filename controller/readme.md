# Control Law
This directory contains the control law implementations.

## State Machine
The state machine (implemented in `state_machine.py`) determines the current flight mode of the glider. It listens to the estimator output and determines the appropriate law to use based on the glider's state and environment.
- Cruise: The glider flies straight ahead towards an objective waypoint. A feedback controller adjusts the bank angle to track a desired heading.
- Probe: The glider performs a series of predetermined maneuvers to search for thermals. TBD: Circles? Figure-eights? Zig-zags?
- Thermal: The glider circles to stay within a thermal updraft. A feedback controller adjusts the bank angle to maintain a desired turn radius around the estimated thermal core, with the predicted thermal strength and radius used to optimize the circling performance via optimal speed and bank angle.

## Waypoint Control Law
The waypoint control law (implemented in `waypoint_control.py`) is a simple pure pursuit controller that guides the glider towards a specified waypoint. The target waypoint is assumed to be computed by a higher-level mission planner and passed into the controller. The controller computes the desired arc to the waypoint via a lookahead distance and adjusts the bank angle to follow that arc.

Note that the waypoint controller computes everything with respect to the ground frame.

# TODO
In cruise mode, consider flying a MacCready speed based on the estimated average thermal strength and environmental sink rate.