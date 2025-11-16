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

- In steady, coordinated flight, the aircraft’s angle of attack ($\alpha$) is largely determined by its pitch attitude ($\theta$) and flight path angle, rather than being an independent state variable. In other words, $\alpha$ varies slowly and predictably with $\theta$ for small perturbations around trim. Because the body-rate controller operates at a much faster timescale than changes in $\alpha$, and because $\alpha$’s influence on aerodynamic derivatives is already captured through the dependence on airspeed (via dynamic pressure) and trim condition, we can omit $\alpha$ from the state vector and treat its effect as being absorbed into the gain-scheduling over operating points.

- However, even within this simplified representation, the trim bank angle $\phi$ (or equivalently, the associated load factor $n_z$ in a coordinated turn) changes the trim body rates $(p_0, q_0, r_0)$ and therefore the linearized body-rate dynamics through the inertial coupling terms in the equations of motion. To capture both straight–and–level and banked coordinated turn behavior, we treat the identified $(A, B)$ matrices as weakly dependent on bank angle and perform system identification at multiple trim conditions (e.g., $\phi \approx 0^\circ$ and a representative turning bank angle).


### Bank Angle and Coordinated Turn Kinematics

The above model focuses on the body angular rates $\omega = [\,p,\; q,\; r\,]$ as the primary states of interest. An important subtlety is that, even when sideslip ($\beta$) and angle of attack ($\alpha$) are not explicitly included as states, the trim values of $(p, q, r)$ (and thus, the linearized dynamics) still depend on the bank angle $\phi$ through coordinated–turn kinematics.

For a steady, level, coordinated turn at constant airspeed $V$ and bank angle $\phi$ (with $\beta \approx 0$ and constant altitude), the required centripetal acceleration is provided by the horizontal component of lift. This yields the familiar relationship between yaw (heading) rate and bank angle:

$$
\dot{\psi} \approx \frac{g \tan \phi}{V}
$$

Expressed in the body frame, this leads to nonzero trim body rates,
$(p_0, q_0, r_0)$, even though the motion is “steady” in an inertial sense. To first order (for small pitch angles),
the trim yaw rate is approximately

$$
r_0(\phi) \;\approx\; \frac{g \sin \phi}{V}
$$

and there is a smaller but nonzero trim pitch rate $q_0(\phi)$ due to the geometry of the turn and the flight–path angle. In straight–and–level flight ($\phi \approx 0$), we have

$$
p_0 \approx 0, \quad q_0 \approx 0, \quad r_0 \approx 0,
$$

whereas in a coordinated turn at $\phi \neq 0$ the trim body rates are nonzero.

Returning to the body–frame equations of motion,

$$
\begin{aligned}
J_x \dot{p} &= (J_y - J_z)\, q r + l \\
J_y \dot{q} &= (J_z - J_x)\, p r + m \\
J_z \dot{r} &= (J_x - J_y)\, p q + n,
\end{aligned}
$$

we observe that the inertial coupling terms (the products $p q$, $p r$, $q r$) are evaluated at the trim rates $(p_0, q_0, r_0)$ when we linearize about a trim condition. In straight–and–level flight, these products are approximately zero, and the corresponding inertial coupling matrix in the linearized $A$ matrix is negligible. In a coordinated turn, however, the nonzero trim rates induce additional coupling terms in $A$ that **do not appear** in a straight–and–level linearization.

As a result, a body–rate model identified purely from straight–and–level data will generally **not** capture the dynamics observed in a banked, coordinated turn at the same airspeed. To obtain a model that is valid across both regimes, we must treat the system as **parameter–varying in bank angle** (or equivalently, in load factor $n_z$) and perform system identification at both straight–and–level and banked trim conditions.


### Model-to-fit
With the above assumptions and simplifications, we can define the model to fit as follows:
$$

\begin{bmatrix}
\Delta p_{k+1} \\
\Delta q_{k+1} \\
\Delta r_{k+1}
\end{bmatrix}
=
A
\begin{bmatrix}
\Delta p_{k} \\
\Delta q_{k} \\
\Delta r_{k}
\end{bmatrix}
+
B
\begin{bmatrix}
\Delta \delta_{a_{k}} \\
\Delta \delta_{e_{k}} \\
\Delta \delta_{r_{k}}
\end{bmatrix}
$$

Where:
- $A$ is a 3x3 matrix representing the state transition dynamics of body angular rates.
- $B$ is a 3x3 matrix representing the control effectiveness of the control surfaces on body angular rates.
- $\Delta p$, $\Delta q$, $\Delta r$ are the deviations of body angular rates from trim.
- $\Delta \delta_a$, $\Delta \delta_e$, $\Delta \delta_r$ are the deviations of control surface deflections from trim.

The optimization objective during system identification can be formally defined as:
$$
\min_{L, M, N} \sum_{k=0}^{j - 1} \left( \Delta \omega_{k+1} - A \Delta \omega_k - B \Delta \delta_k \right)^2
$$

Where:
- $j$ is the total number of data samples collected during the system identification experiment.
- $\Delta \omega_{k+1}$ is the measured angular acceleration at sample $k+1$.
- $\Delta \omega_k$ is the measured angular rate deviation at sample $k$.
- $\Delta \delta_k$ is the measured control surface deviation at sample $k$.

## Identification Procedure
### Trim Condition Setup
In order to develop the small-signal linear model, we first need to establish a trim condition for the aircraft (mathematically, a point where $f(x_{trim}, u_{trim}) = 0$). 

The method chosen for this procedure is to have a coarsely-tuned PID controller target a specific pitch angle $\theta_{trim}$ and a specific roll angle $\phi_{trim}$, while maintaining a sideslip angle of $\beta = 0 \ rad$ (corresponding to coordinated flight). The gains for this PID controller need not be perfect (or have any real basis in flight dynamics theory, aka a "yolo-tune) - they just need to be good enough to maintain a steady attitude for the purposes of data collection.

The PID controller follows the following difference equation form
$u[k] = K_p e[k] + K_i \sum_{n=0}^{k} e[n] + K_d (e[k] - e[k-1])$
Where:
- $u[k]$ is the control output at time step $k$ (which maps to control surface deflections)
- $e[k]$ is the error signal at time step $k$ (difference between desired and actual attitude)
- $K_p$, $K_i$, $K_d$ are the proportional, integral, and derivative gains, respectively.

We consider at least two representative trim conditions at a given airspeed:

1. **Straight-and-Level (S&L) Trim**  
   A coarse PID (or attitude-hold) controller is used to:
   - target a specific pitch angle $\theta_{trim}$,
   - maintain roll angle $\phi_{trim} \approx 0$,
   - drive sideslip $\beta \approx 0$,
   - hold airspeed and altitude approximately constant.

2. **Coordinated Turn Trim**  
   To capture the effect of bank-angle-dependent kinematics on the body-rate dynamics, we also establish a trim corresponding to a steady, coordinated turn at a representative bank angle (e.g., $\phi_{trim} \approx 30^\circ$):
   - roll angle $\phi_{trim} \neq 0$ (constant bank),
   - sideslip $\beta \approx 0$ (coordinated),
   - airspeed and altitude approximately constant.

Once the aircraft is stabilized at the desired attitude, we log the steady-state control surface deflections ($\delta_{aileron, trim}$, $\delta_{elevator, trim}$, $\delta_{rudder, trim}$), as well as the corresponding trim body rates ($p_0$, $q_0$, $r_0$).

### Perturbation Input
Once the aircraft is in a trimmed condition, we apply a series of perturbation inputs per control surface to excite the body angular rate dynamics. The perturbation input follows the following general form:
- Step Input: A step change in control surface deflection, exercising positive and negative deflection.
- Random Binary Sequence (RBS): A pseudo-random sequence of control surface deflection, switching between positive and negative deflections at a specified frequency.

For each trim condition, we record time histories of:
- body rates $[p, q, r]$,
- their finite difference approximations.
- control surface deflections $[\delta_a, \delta_e, \delta_r]$,

Once the above steps are completed, the collected data is used to solve the optimization problem defined earlier, yielding the identified state-space matrices A and B for the body-rate dynamics.
