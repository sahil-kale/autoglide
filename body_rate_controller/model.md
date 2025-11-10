# Body-Rate Model Identification

## Body Frame Axes:
See [here](https://jsbsim-team.github.io/jsbsim-reference-manual/user/concepts/frames-of-reference/) for a description of the body frame convention used in this module.
- X-axis: Points forward along the fuselage of the aircraft
- Y-axis: Points out the right wing of the aircraft
- Z-axis: Points downward, completing the right-handed coordinate system

The rotations about these axes are defined as:
$\boldsymbol{\omega} = [\,p,\; q,\; r\,]$

- Roll ($p$): Rotation about the X-axis
- Pitch ($q$): Rotation about the Y-axis
- Yaw ($r$): Rotation about the Z-axis*

- Roll ($p$): Rotation about the X-axis
- Pitch ($q$): Rotation about the Y-axis
- Yaw ($r$): Rotation about the Z-axis*

## Control Surfaces:
- $\delta_{aileron}$: Aileron deflection (normalized -1 to 1, where positive deflects right aileron up, inducing roll to the right)
- $\delta_{elevator}$: Elevator deflection (normalized -1 to 1, where positive deflects elevator up, inducing pitch up)
- $\delta_{rudder}$: Rudder deflection (normalized -1 to 1, where positive deflects rudder to the right, inducing yaw to the right)

## Body Frame Equations of Motion
The following equations, taken from [Aircraft Dynamics and Automatic Control by Stevens, Lewis and Johnson (3rd Edition)](https://www.amazon.com/Aircraft-Control-Simulation-Dynamics-Autonomous/dp/1118870980), describe the rotational dynamics of an aircraft within the body frame (equations 1.7-8):

$\sum \Tau_{body-x} =  J_x \dot{p} = (J_y - J_z)(q r) + l$

$\sum \Tau_{body-y} =  J_y \dot{q} = (J_z - J_x)(p r) + m$

$\sum \Tau_{body-z} =  J_z \dot{r} = (J_x - J_y)(p q) + n$


Where:
- $J_x$, $J_y$, $J_z$ are the moments of inertia about the body frame axes
- $l$, $m$, $n$ are the aerodynamic moments about the **body frame** axes, which are functions of the control surface deflections ($\delta_{aileron}$, $\delta_{elevator}$, $\delta_{rudder}$).

### Wind Frame Moment Definitions
The moments $l$, $m$, and $n$ are originally defined about the wind frame axes, and must be transformed to the body frame axes ($C_{wind\_to\_body} = f(\alpha, \beta)$) in order to be used in the above equations of motion. The moments about the body frame are as follows:

Let us define dynamic pressure as $\bar{q} = \frac{\rho V^2}{2}$, where $\rho$ is air density ($\frac{kg}{m^3}$) and $V$ is airspeed in the wind frame ($\frac{m}{s}$). Let $S$ be the wing reference area ($m^2$), $b$ be the wingspan ($m$), $M$ be the mach number.

#### Roll Moment ($l$):
Roll moment about the wind frame x-axis is defined as:

$l_{wind} = \bar{q} S b C_l(\alpha, \beta, M, \delta_{aileron}, \delta_{rudder})$

Where $C_l = C_{l_0}(\alpha, \beta, M) + C_{l_{\delta_{aileron}}}(\alpha, \beta, M, \delta_{aileron}) + C_{l_{\delta_{rudder}}}(\alpha, \beta, M, \delta_{rudder}) + \frac{b}{2V}( C_{l_p}(\alpha, \beta, M) p + C_{l_r}(\alpha, \beta, M) r )$
- $C_{l_0}$ is the zero-lift roll moment coefficient (meant to account for asymmetries in the aircraft, such as engine placement or dihedral angle). note that in a trimmed condition, this should be $\approx 0$.
- $C_{l_{\delta_{aileron}}}$ is the roll moment coefficient due to aileron deflection. In a linearized model, this is typically modeled as $C_{l_{\delta_{aileron}}} \approx C_{l_{\delta_{aileron}}}(\alpha, \beta, M) \cdot \delta_{aileron}$. This is the primary control input for roll control, which is mathematically telling us that deflecting the ailerons will induce a roll moment by altering the lift distribution across the wings.
- $C_{l_{\delta_{rudder}}}$ is the roll moment coefficient due to rudder deflection. This term captures the secondary effect of rudder deflection on roll moment (primarily through yaw-roll coupling). In a linearized model, this is typically modeled as $C_{l_{\delta_{rudder}}} \approx C_{l_{\delta_{rudder}}}(\alpha, \beta, M) \cdot \delta_{rudder}$.
- $\frac{b}{2V}( C_{l_p}(\alpha, \beta, M) p + C_{l_r}(\alpha, \beta, M) r )$ is the roll damping term, which captures the effect of roll rate ($p$) on the roll moment. This term is crucial for stability analysis and control design, as it represents the natural damping effect that opposes roll motion.

#### Pitch Moment ($m$):
Pitch moment about the wind-frame **y-axis** is defined as:

$m_{\text{wind}} \;=\; \bar{q}\, S\, \bar{c}\; C_m(\alpha,\beta,M,\delta_{elevator},\delta_{aileron})$

Where
$C_m = C_{m_0}(\alpha,\beta,M) + C_{m_{\delta_{elevator}}}(\alpha, \beta, M, \delta_{elevator}) + \frac{\bar{c}}{2V} (C_{m_q}(\alpha, \beta, M) q + C_{m_{\alpha}}(\alpha, \beta, M) \dot{\alpha})$

- $C_{m_0}$ is the baseline (trim) pitch-moment coefficient as a function of state; near a properly trimmed condition this is $\approx 0$.
- $C_{m_{\delta_{elevator}}}$ is the elevator control effectiveness. In a linearized model it appears as $C_{m_{\delta_{elevator}}}^{linear}\,\delta_{elevator}$ and is the **primary** control input for pitch.
- $\frac{\bar{c}}{2V} (C_{m_q}(\alpha, \beta, M) q + C_{m_{\alpha}}(\alpha, \beta, M) \dot{\alpha})$ is the pitch damping term, which captures the effect of pitch rate ($q$) and angle of attack rate ($\dot{\alpha}$) on the pitch moment. This term is crucial for stability analysis and control design, as it represents the natural damping effect that opposes pitch motion.
- $\bar{c}$ is the mean aerodynamic chord (m).

#### Yaw Moment ($n$):
Yaw moment about the wind-frame **z-axis** is defined as:

\[
n_{\text{wind}} \;=\; \bar{q}\, S\, b\; C_n(\alpha,\beta,M,\delta_{rudder},\delta_{aileron})
\]

Where
\[
C_n \;=\; C_{n_0}(\alpha,\beta,M)
\;+\; C_{n_{\delta_{rudder}}}(\alpha,\beta,M)\,\delta_{rudder}
\;+\; C_{n_{\delta_{aileron}}}(\alpha,\beta,M)\,\delta_{aileron}
\;+\; \frac{b}{2V}\, C_{n_r}(\alpha,\beta,M)\, r.
\]

- \(C_{n_0}\) is the baseline yaw-moment coefficient (includes effects like vertical tail/fin and static asymmetries); \(\approx 0\) at coordinated trim.
- \(C_{n_{\delta_{rudder}}}\) is the rudder control effectiveness. In a linearized model it appears as \(C_{n_{\delta_{rudder}}}\,\delta_{rudder}\) and is the **primary** yaw control channel (used for \(\beta\to 0\)).
- \(C_{n_{\delta_{aileron}}}\) captures **adverse yaw** (aileron deflection producing yaw via differential drag/lift); often non-negligible and a key cross-coupling in roll–yaw dynamics.
- \(\frac{b}{2V}\, C_{n_r}\, r\) is the **yaw-rate damping** term (stabilizing for \(C_{n_r}<0\)), opposing yaw motion.



## Body Frame System Identification
In order to design and tune the body-rate controller, a system identification procedure is required to derive a small-signal linear model of the aircraft dynamics in the body frame. 

### Use of data-driven system identification
In order to tune the body-rate controller and verify stability, a system identification procedure is required for the plant under control. While there are several mathematical models that can provide close to exact representations of the aircraft dynamics, we opt for a data-driven approach that captures the true dynamics of the aircraft in the simulation environment. The rationale for doing so is two-fold:
- The environment may have unmodeled dynamics or nonlinearities that are not captured in the mathematical models. Having an explicit system identification procedure ensures that these dynamics are captured in the identified model, regardless of the environment complexity.
- It eliminates the need for deep expertise in flight dynamics and control theory to derive the mathematical models, making it easier for new users to get started with the control stack.

### System Identification Objective
Here, we formally define the objective of the system identification procedure for the body-rate controller. Generically, within the body frame, the dynamics of the aircraft within the body frame can be described as:
$$
\dot{x} = f(x, u)
$$

Where:
- $x$ is the state vector, which includes body angular rates ($\omega = [\,p,\; q,\; r\,]$), and other relevant state (ex, dynamic pressure, which affects control surface effectiveness)
- $u$ is the control input vector, which includes control surface deflections ($\delta_{aileron}$, $\delta_{elevator}$, $\delta_{rudder}$)
- $f$ is a nonlinear function that describes the aircraft dynamics within the body frame.

