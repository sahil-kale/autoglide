# Learnings
Unstructured learnings so I don't lose them for later in full writeup.

- Regularization is important to prevent wild jumps in estimates. This massively improved the stability of the estimator (before, it would jump around a lot and be very noisy).
- Optimizer confidence can be estimated based on the variance of the residuals. If the variance is low, we can be more confident in the estimates. This can be used to adjust the regularization strength dynamically.
- Intuition about the system in general (glider dynamics, thermal dynamics) proved to be super useful in generally understanding the math and what was going on.

## Optimization Lessons Learned
I learned very quickly that regularization was key to getting the optimization to behave well. Without it, the estimates would jump around wildly between iterations, and it wasn't uncommon to have blips where the optimizer would solve to a completely incorrect solution with a different center, even though it tracked fine before. It was cool to see how adding the terms made the estimates much more stable at the expense of some phase lag in the estimates (basically, a low-pass filter effect).

Also, unlike a bayesian filter, the optimization doesn't provide any uncertainty estimates on the parameters. I ended up having to come up with my own measure of uncertainty, which attempts to match how well the model fits the observed variometer data. This is still a little rough around the edges, in particular when flying in complete sink (the model is technically right in saying that the thermal location is where it was last seen, but at that point the uncertainty has very little meaning).

### Observability
I came back to this problem a little bit later and realized that I have the power to formally analyze the observability of the system with both techniques!

Let us define the state vector as:
$$
\theta = \begin{bmatrix} W_0 \\ R_{th} \\ x_c \\ y_c \end{bmatrix}
$$

The measurement model is given by a simple Gaussian thermal model:
$$
h(\theta, x, y) = W_0 \exp\left(-\frac{(x - x_c)^2 + (y - y_c)^2}{R_{th}^2}\right)
$$
Where $h(\theta, x, y)$ is the measurement model which predicts the vertical air velocity (which is measured by a variometer, $w_{meas}$) at position $(x, y)$ given the thermal parameters $\theta$. $x, y$ are the glider's positions, while $W_0, R_{th}, x_c, y_c$ are the thermal parameters to be estimated.

The gradient of the measurement model with respect to the state vector is calculated in `observability.py`. But for this discussion, understanding that $H$ is the Jacobian matrix of partial derivatives in R4 is sufficient.

#### EKF Observability
Whenever the Kalman filter updates, it computes the Kalman gain $K$ as:
$$K = P H^T (H P H^T + R)^{-1}$$
Where $P$ is the state covariance matrix, $H$ is the measurement Jacobian, and $R$ is the measurement noise covariance.
For a single measurement, note that the second term is just a scalar, meaning the Kalman gain is just a scaled version of $P H^T$.

On its own, this means the Kalman gain just moves in a single direction in R4, meaning only one degree of freedom is updated at a time. This is a problem because the thermal parameters are highly coupled - for example, if the thermal core moves slightly, the strength and radius may also need to adjust to fit the measurements well, but the EKF can only adjust the parameters across a single vector in 4D space at a time as it only receives one measurement at a time (at least, the way I implemented it).

#### Nonlinear Optimization Observability
I didn't go through this as a proof since it did end up working, but the intuition is that the optimization has access to a window of measurements at once, meaning it can adjust the parameters in a way that fits all the measurements at once. This means it can adjust all 4 parameters at once in a way that minimizes the overall residuals, rather than being limited to a single direction in 4D space at a time.
