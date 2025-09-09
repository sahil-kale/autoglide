# Thermal Estimator
The goal of the thermal estimator model is to estimate the thermal core position and strength based on the glider's positions and vertical air velocity measurements. It uses an Extended Kalman Filter (EKF) to perform the estimation.

## State Nomenclature
The state vector is defined as:
$$x = \begin{bmatrix} x_c \\ y_c \\ W_0 \\ R_th \end{bmatrix}$$
Where:
- $x_c, y_c$ are the thermal core coordinates
- $W_0$ is the thermal strength (maximum vertical air velocity at the core)
- $R_th$ is the thermal radius (the distance from the core where the vertical air velocity drops to zero)

## Process Model
The process model assumes that the thermal core position and strength are constant over time, with some process noise. As a result, there is no actual dynamics in the state transition, and the state at the next time step is equal to the current state plus some process noise.

## Update Model
The update model assumes that a variometer measurement is taken at each time step. For context, a variometer measures the vertical velocity of the airmass the glider is flying through, which is how a thermal's presence can be inferred. The variometer has a lot of noise, so the filter must be robust to this. The measurement model is defined as:
$$
z = h(W0, R_th, x, y) 
$$

For now, a simple Gaussian thermal model is used. A more complex model could be implemented later if desired, but this captures the core dynamics of a thermal.

$$
h(W0, R_th, x, y) = W0 * exp(-\rho), where \rho = \frac{(x - x_c)^2 + (y - y_c)^2}{R_th^2}
$$


## Misc
### EKF Equations
Key EKF equations:
Prediction step:
$$
\hat{x}_{k|k-1} = f(\hat{x}_{k-1|k-1}, u_{k-1})
$$
$$
P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k
$$

Update step:
$$
K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}
$$
$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - h(\hat{x}_{k|k-1}))
$$