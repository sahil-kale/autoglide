# Control Law
This directory contains the control law implementations.

## State Machine
The state machine (implemented in `state_machine.py`) determines the current flight mode of the glider. It listens to the estimator output and determines the appropriate law to use based on the glider's state and environment.
- Cruise: The glider flies straight ahead towards an objective waypoint.
- Probe: The glider performs a series of predetermined maneuvers to search for thermals. TBD: Circles? Figure-eights? Zig-zags?
- Thermal: The glider circles to stay within a thermal updraft.

All control modes use the same 

## L1 Guidance Control Law
The L1 guidance control law (implemented in `l1_guidance_law.py`) enables the glider to track arbitrary paths (expressed as waypoints). A good explanation of L1 guidance can be found in this [paper](https://mercury.kau.ac.kr/park/Archive/PCUAV/gnc_park_deyst_how.pdf). The key idea is that an acceleration command $a_s_{cmd}$ is computed based on the lateral error to the desired path's lookahead point. The below explanation is my summary of the key points from the paper that I found relevant in implementing the law.

### Key Equations
The lateral acceleration command is given by:
$$
a_{s_{cmd}} = 2 \frac{V^2}{L_1} \sin(\eta)
$$
Where:
- $V$ is the current ground speed of the glider
- $L_1$ is the lookahead distance (a tunable parameter)
- $\eta$ is the angle between the glider's velocity vector and the vector from the glider to the lookahead point on the path

Note that this same law can be used to track straight lines and circular arcs. The difference is in how the lookahead point is computed.
For circular arcs, the paper proposes the following method to compute the lookahead point:
The centripetal acceleration required to follow a circular path of radius $R$ at speed $V$ is given by $a_c = \frac{V^2}{R}$. The lateral acceleration command can be set to this value to follow the arc. Rearranging the equation gives the required turn radius:
$$ 
\frac{V^2}{R} = 2 \frac{V^2}{L_1} \sin(\eta) \implies R = \frac{L_1}{2 \sin(\eta)}
$$

### Straight Line Path Tracking
For a straight line, the cross-track error $d$ is minimized by the above control law in a fashion similar to a gain-scheduled PD controller (equation 3). The gain scheduling comes from the fact that the velocity $V$ and lookahead distance $L_1$ can vary. For small angles, $\sin(\eta) \approx \eta$, and the acceleration command approximates to:
$$
a_{s_{cmd}} \approx 2 \frac{V^2}{L_1^2} (\frac{d}{L_1} + \frac{\dot{d}}{V})
$$

Where
- $d$ is the cross-track error (lateral distance from the glider to the path)
- $\dot{d}$ is the rate of change of the cross-track error

The paper goes into more detail with regards to the stability analysis of this control law (particularly when the inner-loop dynamics are negligible, which is assumed for this guidance law) that is slightly underdamped ($zeta = 1/\sqrt{2}$).

### Straight Line with Perturbations
The analysis in the paper demonstrates that the control law behaves similar to the above PD controller on lateral offset when faced with perturbations in the path to track, although with a low-pass filter effect on the perturbations. This is important as it means that the glider can track a straight line path even in the presence of disturbances, whether these be due to wind gusts or trajectory perturbations.

## Bank Angle Mapping
Within the glider kinematic model, we see that lateral acceleration is related to the roll angle $\phi$ by:
$$
a_s = g \tan(\phi)
$$

Therefore, the bank angle command can be computed from the lateral acceleration command by:
$$
\phi_{cmd} = \tan^{-1}\left(\frac{a_{s_{cmd}}{g}\right)
$$ 

# TODO
In cruise mode, consider flying a MacCready speed based on the estimated average thermal strength and environmental sink rate.