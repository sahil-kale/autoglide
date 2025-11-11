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

$\sum \tau_{body-x} =  J_x \dot{p} = (J_y - J_z)(q r) + l$

$\sum \tau_{body-y} =  J_y \dot{q} = (J_z - J_x)(p r) + m$

$\sum \tau_{body-z} =  J_z \dot{r} = (J_x - J_y)(p q) + n$


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
- $C_{m_{\delta_{elevator}}}$ is the elevator control effectiveness. In a linearized model it appears as $C_{m_{\delta_{elevator}}}(\alpha, \beta, h)^{linear}\,\delta_{elevator}$ and is the **primary** control input for pitch.
- $\frac{\bar{c}}{2V} (C_{m_q}(\alpha, \beta, M) q + C_{m_{\alpha}}(\alpha, \beta, M) \dot{\alpha})$ is the pitch damping term, which captures the effect of pitch rate ($q$) and angle of attack rate ($\dot{\alpha}$) on the pitch moment. This term is crucial for stability analysis and control design, as it represents the natural damping effect that opposes pitch motion.
- $\bar{c}$ is the mean aerodynamic chord (m).

#### Yaw Moment ($n$):
Yaw moment about the wind-frame **z-axis** is defined as:

$C_n = C_{n_0}(\alpha, \beta, M, T_c) + C_{n_{\delta_r}}(\alpha, \beta, M, \delta_r) + C_{n_{\delta_a}}(\alpha, \beta, M, \delta_a) + \frac{b}{2V_T} (C_{n_p}(\alpha, M)P + C_{n_r}(\alpha, M)R)$

- $C_{n_0}$ is the baseline (trim) yaw-moment coefficient as a function of state and throttle command; near a properly trimmed condition this is $\approx 0$.
- $C_{n_{\delta_r}}$ is the rudder control effectiveness. In a linearized model it appears as $C_{n_{\delta_r}}(\alpha, \beta, M)^{linear}\,\delta_{rudder}$ and is the **primary** control input for yaw.
- $C_{n_{\delta_a}}$ is the secondary yaw moment due to aileron deflection. In a linearized model it appears as $C_{n_{\delta_a}}(\alpha, \beta, M)^{linear}\,\delta_{aileron}$ and captures the yaw moment induced by aileron deflection (primarily through roll-yaw coupling).
- $\frac{b}{2V} (C_{n_p}(\alpha, M) p + C_{n_r}(\alpha, M) r)$ is the yaw damping term, which captures the effect of yaw rate ($r$) on the yaw moment. This term is crucial for stability analysis and control design, as it represents the natural damping effect that opposes yaw motion.

## Body Frame System Identification
In order to design and tune the body-rate controller, a system identification procedure is required to derive a small-signal linear model of the aircraft dynamics in the body frame. 

### Use of data-driven system identification
In order to tune the body-rate controller and verify stability, a system identification procedure is required for the plant under control. While there are several mathematical models that can provide close to exact representations of the aircraft dynamics, we opt for a data-driven approach that captures the true dynamics of the aircraft in the simulation environment. The rationale for doing so is two-fold:
- The environment may have unmodeled dynamics or nonlinearities that are not captured in the mathematical models. Having an explicit system identification procedure ensures that these dynamics are captured in the identified model, regardless of the environment complexity.
- It eliminates the need for deep expertise in flight dynamics and control theory to derive the mathematical models, making it easier for new users to get started with the control stack.

### System Identification Objective
Here, we formally define the objective of the system identification procedure for the body-rate controller. Generically, within the body frame, the dynamics of the aircraft within the body frame can be described as: $\dot{x} = f(x, u)$

Where:
- $x$ is the state vector, which includes body angular rates ($\omega = [\,p,\; q,\; r\,]$), and other relevant state (ex, dynamic pressure, which affects control surface effectiveness)
- $u$ is the control input vector, which includes control surface deflections ($\delta_{aileron}$, $\delta_{elevator}$, $\delta_{rudder}$)
- $f$ is a nonlinear function that describes the aircraft dynamics within the body frame.

### Linearization About Trim Condition
For the purposes of controller design and stability analysis, it is in our interest to derive a linear approximation of the aircraft dynamics about a trim condition ($x_{trim}$, $u_{trim}$). This linear approximation can be expressed as (in the familiar state-space form): $\dot{x} = A \Delta x + B \Delta u$, around a trim point $f(x_{trim}, u_{trim}) = 0$

Where:
- $A$ is the state transition matrix, which describes how the state evolves over time in the absence of control inputs.
- $B$ is the control input matrix, which describes how the control inputs affect the state.
- $\Delta x = x - x_{trim}$ is the deviation of the state from the trim condition.
- $\Delta u = u - u_{trim}$ is the deviation of the control inputs from the trim condition.

### Model Simplifications
From the above equations, we can make several simplifications to the model in order to focus on the body angular rates ($\omega = [\,p,\; q,\; r\,]$) as the primary states of interest for the body-rate controller.
- We can conclude that the velocity state has a non-trivial impact on control surface effectiveness, as it's directly related to dynamic pressure ($\bar{q} = \frac{\rho V^2}{2}$). This implies that the identified model will be valid only within a certain range of airspeeds (or dynamic pressures). As a result, we may need to perform system identification at multiple trim conditions to cover the full flight envelope, and schedule a controller gain based on airspeed or dynamic pressure. In effect, the system is linear parameter varying (LPV), but we can treat it as a set of LTI systems at various operating points and remove the dependency on velocity from the state vector.
- The body-rate controller within this autopilot is intended to operate within a coordinated flight regime, where sideslip ($\beta$) is maintained at or close to zero. As a result, we can remove sideslip from the state vector, and treat it as a disturbance that is rejected by the outer-loop controller.
- In steady, coordinated flight, the aircraft’s angle of attack ($\alpha$) is largely determined by its pitch attitude ($\theta$) and flight path angle, rather than being an independent state variable. In other words, $\alpha$ varies slowly and predictably with $\theta$ for small perturbations around trim. Because the body-rate controller operates at a much faster timescale than changes in $\alpha$, and because $\alpha$’s influence on aerodynamic derivatives is already captured through the dependence on airspeed (via dynamic pressure), we can effectively absorb its variation into the same gain-scheduling framework used for velocity. This allows us to omit $\alpha$ from the state vector without losing meaningful fidelity for inner-loop control design.

### Model-to-fit
With the above assumptions and simplifications, we can define the model to fit as follows:
$$

\begin{bmatrix}
\Delta \dot{p} \\
\Delta \dot{q} \\
\Delta \dot{r}
\end{bmatrix}
=
\underbrace{
\begin{bmatrix}
L_p & 0 & 0 \\
0 & M_q & M_r \\
0 & N_q & N_r
\end{bmatrix}
}_{A}
\begin{bmatrix}
\Delta p \\
\Delta q \\
\Delta r
\end{bmatrix}
+
\underbrace{
\begin{bmatrix}
L_{\delta_a} & 0 & 0 \\
0 & M_{\delta_e} & M_{\delta_r} \\
0 & N_{\delta_e} & N_{\delta_r}
\end{bmatrix}
}_{B}
\begin{bmatrix}
\Delta \delta_a \\
\Delta \delta_e \\
\Delta \delta_r
\end{bmatrix}

$$

Where:
- $L_p$, $M_q$, $M_r$, $N_q$, $N_r$ are the stability derivatives to be identified.
- $L_{\delta_a}$, $M_{\delta_e}$, $M_{\delta_r}$, $N_{\delta_e}$, $N_{\delta_r}$ are the control effectiveness derivatives to be identified.
- $\Delta p$, $\Delta q$, $\Delta r$ are the deviations of body angular rates from trim.
- $\Delta \delta_a$, $\Delta \delta_e$, $\Delta \delta_r$ are the deviations of control surface deflections from trim.

Note the 0'ed out terms in the A and B matrices, which stem from the assumptions made earlier (specifically, lack of AoA and sideslip states accounted for in the model), and the core equations of motion that govern the body angular rates (ex: the pitch rate $\dot{q}$ is not directly affected by aileron deflection, hence the 0 in that position in the B matrix).

The optimization objective during system identification can be formally defined as:
$$
\min_{L, M, N} \sum_{i=1}^{j} \left( \Delta \dot{\omega}_i - A \Delta \omega_i - B \Delta \delta_i \right)^2
$$

Where:
- $j$ is the total number of data samples collected during the system identification experiment.
- $\Delta \dot{\omega}_i$ is the measured angular acceleration at sample $i$.
- $\Delta \omega_i$ is the measured angular rate deviation at sample $i$.
- $\Delta \delta_i$ is the measured control surface deviation at sample $i$.

## Identification Procedure
### Trim Condition Setup
In order to develop the small-signal linear model, we first need to establish a trim condition for the aircraft (mathematically, a point where $f(x_{trim}, u_{trim}) = 0$). 

The method chosen for this procedure is to have a coarsely-tuned PID controller target a specific pitch angle $\theta_{trim}$, while maintaining a roll angle $\phi_{trim} = 0 \ rad/s$ and sideslip $\beta = 0 \ rad$. The gains for this PID controller need not be perfect (or have any real basis in flight dynamics theory, aka a "yolo-tune) - they just need to be good enough to maintain a steady attitude for the purposes of data collection.

The PID controller follows the following difference equation form
$u[k] = K_p e[k] + K_i \sum_{n=0}^{k} e[n] + K_d (e[k] - e[k-1])$
Where:
- $u[k]$ is the control output at time step $k$ (which maps to control surface deflections)
- $e[k]$ is the error signal at time step $k$ (difference between desired and actual attitude)
- $K_p$, $K_i$, $K_d$ are the proportional, integral, and derivative gains, respectively.

Once the aircraft is stabilized at the desired attitude, we log the steady-state control surface deflections ($\delta_{aileron, trim}$, $\delta_{elevator, trim}$, $\delta_{rudder, trim}$).

### Perturbation Input
Once the aircraft is in a trimmed condition, we apply a series of perturbation inputs to excite the body angular rate dynamics. The perturbation input follows the following general form:
- Step Input: A step change in control surface deflection, exercising positive and negative deflection.
- Random Binary Sequence (RBS): A pseudo-random sequence of control surface deflections, switching between positive and negative deflections at a specified frequency.

Once the above steps are completed, the collected data is used to solve the optimization problem defined earlier, yielding the identified state-space matrices A and B for the body-rate dynamics.
