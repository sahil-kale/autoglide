# Thermal Estimator
The goal of the thermal estimator model is to estimate the thermal core position and strength based on the glider's positions and vertical air velocity measurements. It uses an optimizer to minimize the difference between predicted and measured vertical air velocities.

## Optimization Variables
The variables optimized over are:
- $x_c, y_c$, the thermal core coordinates
- $W_0$, the thermal strength (maximum vertical air velocity at the core)
- $R_th$, the thermal radius (the distance from the core where the vertical air velocity drops to zero)

## Optimization Problem
The estimator uses a nonlinear optimization approach to minimize the residuals between the predicted and measured vertical air currents. The predicted vertical air velocity at the glider's position is given by a Gaussian thermal model:
$$
w_{pred}(x, y, x_c, y_c, W_0, R_{th}) = W_0 \exp\left(-\frac{(x - x_c)^2 + (y - y_c)^2}{R_{th}^2}\right)
$$

The cost function to minimize is the sum of squared differences between the predicted and measured vertical air velocities:
$$
J = \sum_{i=0}^{N - 1} \left(w_{pred}(x_i, y_i, x_c, y_c, W_0, R_{th}) - w_{meas,i}\right)^2 
$$
Where $(x_i, y_i)$ are the glider's positions and $w_{meas,i}$ are the corresponding measured vertical air velocities (measured from variometer), with N being the total number of variometer measurements used in the estimation.

## Notes
Consider optional regularization terms to prevent overfitting, especially if the number of measurements is low. Constraints on the variables (e.g., $W_0 > 0$, $R_{th} > 0$) can also be added to ensure physically meaningful estimates. Further, consider penalizing large changes in estimates between time steps to promote smoother estimates over time.