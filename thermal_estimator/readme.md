# Thermal Estimator
The goal of the thermal estimator model is to estimate the thermal core position and strength based on the glider's positions and vertical air velocity measurements. It uses an optimizer to minimize the difference between predicted and measured vertical air velocities with a simplified Gaussian thermal model. A snippet of the estimator tracking a thermal is shown below:
![Thermal Estimator Tracking](glider_thermal_estimator.png)

## Optimization Variables
The variables optimized over are:
- $x_c, y_c$, the thermal core coordinates
- $W_0$, the thermal strength (maximum vertical air velocity at the core)
- $R_th$, the thermal radius (the distance from the core where the vertical air velocity drops to zero)

## Optimization Problem
The estimator uses a nonlinear optimization approach to minimize the residuals between the predicted and measured vertical air currents. The predicted vertical air velocity at the glider's position is given by a Gaussian thermal model:
Define the radial distance:

$$
r = \sqrt{(x - x_c)^2 + (y - y_c)^2}
$$

Then the predicted vertical velocity is:

$$
w_{\text{pred}}(r, W_0, R_{th}) = W_0 \, \exp\!\left( -\frac{r^2}{R_{th}^2} \right)
$$

The cost function to minimize is the sum of squared differences between the predicted and measured vertical air velocities:

$$
J = \sum_{i=0}^{N-1} \left( w_{pred}(x_i, y_i, x_c, y_c, W_0, R_{th}) - w_{meas,i} \right)^2 + \lambda_1 (W_0 - W_{0,prev})^2 + \lambda_2 (R_{th} - R_{th,prev})^2 + \lambda_3 \left[ (x_c - x_{c,prev})^2 + (y_c - y_{c,prev})^2 \right]
$$


Where $(x_i, y_i)$ are the glider's positions and $w_{meas,i}$ are the corresponding measured vertical air velocities (measured from variometer), with N being the total number of variometer measurements used in the estimation.

The terms with $\lambda_1, \lambda_2, \lambda_3$ are regularization terms to prevent large jumps in the estimated parameters between iterations. The previous estimates $W_{0,prev}, R_{th,prev}, x_{c,prev}, y_{c,prev}$ are used for this purpose. The addition of the regularization terms were found necessary to ensure stable estimates, as without them the estimates would often jump around erratically, causing the controller (which does somewhat blindly trust the estimator) to behave poorly.

## Confidence Metric
The estimator issues a confidence metric based on the residuals of the optimization by running a chi-squared test.
### Chi-Squared Test
The chi-squared statistic attempts to quantify how well the model fits the observed data. In theory, a $\chi^2$ value of 1 indicates that all of the residuals match the expected amount of noise in the measurements. A value much greater than 1 indicates that the model is not fitting the data well (i.e., the residuals are larger than expected), while a value much less than 1 suggests that the model may be overfitting the data (i.e., the residuals are smaller than expected).
The chi-squared statistic is calculated as:

$$
\chi^2 = \frac{1}{N} \sum_{i=0}^{N-1} \left( \frac{ w_{pred}(x_i, y_i, x_c, y_c, W_0, R_{th}) - w_{meas,i} }{ \sigma } \right)^2
$$

Where $\sigma$ is the standard deviation of the measurement noise (assumed to be known), in this case the standard deviation of noise associated with the variometer, and N is the number of measurements used in the estimation.

## Improvements
The simplified estimator model above works well in practice, but there are some improvements:
- Incorporate wind drift and wind gradient into the thermal model to better capture real-world thermal behavior. The simulator does model for thermal core drift with altitude, but the estimator does not currently account for this and requires the optimization problem to be re-solved at each altitude slice.

## The Journey
Before settling on the above approach, I tried an Extended Kalman Filter (EKF) approach to estimate the thermal parameters as part of the state vector. The EKF had access to the actual position of the glider with no actual state dynamics, and the measurement model was the same Gaussian thermal model as above. Truthfully, I didn't have the best reason for using an EKF aside from it being a "first-reach" approach to the problem. When I implemented it, I found the EKF just wouldn't converge and keep increasing its covariance over time. 

I still want to come back and really understand why an EKF wasn't the right choice here (as I found I was using it as a nonlinear optimizer because of the lack of dynamics). My suspicion is that the problem is just not observable enough with the limited measurements available (intuition: only 1 measurement at a time, rather than a window of measurements with associated positions to act on - how can an EKF converge with so little information?). 

Resultingly, I switched to the nonlinear optimization approach above which worked much better in practice. The downside is that it requires a window of measurements to work with, rather than being able to update the estimate at each timestep. 