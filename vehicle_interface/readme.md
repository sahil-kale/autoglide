# Vehicle Interface 
The vehicle interface module abstracts the communication between the simulation environment and the vehicle control system. In particular, it offers a way for the control stack to
- Send control surface commands (aileron deflection, elevator deflection, rudder deflection, spoiler setting)
- Send miscellaneous commands (ex: landing gear position)
- Receive state information as sensed by the vehicle (ex: airspeed, altitude, attitude, position, etc.)
- "Peek" at ground truth data from the simulation environment (ex: exact position, exact attitude, wind vector etc.) for the purposes of logging and debugging

## Convention
See [here](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/#north-oriented-tangent-frames) for a description of the aerospace convention used in this module.