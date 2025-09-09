# Centering Controller
## Defining the Control Problem
The goal of the centering controller is to minimize the distance between the glider's position and the thermal core center, while achieving the highest possible altitude gain rate.

Defining this mathematically:
$$
p = \begin{bmatrix} x \\ y \end{bmatrix}, \quad c = \begin{bmatrix} x_c \\ y_c \end{bmatrix}
$$

Where $p$ is the glider's position and $c$ is the thermal core center position. The centering error, $e$, is defined as $e = p - c$. Let $r = \|e\|$ be the radial distance to the thermal core.

## Lyapunov Candidate
A suitable Lyapunov candidate function for this problem is the squared radial distance to the thermal core:
$V = \frac{1}{2} \|e|^2 = \frac{1}{2} r^2$

Therefore, $\dot{V} = r \cdot \dot{r}$. Since $r > 0$ is always positive, for stability we need $\dot{r} < 0$ as it forces $\dot{V} < 0$.
$r = \sqrt{(x - x_c)^2 + (y - y_c)^2}$
$\dot{r} = V\frac{e^T}{\|e\|}dot{p}$
$\dot{r} = V\frac{e^T}{\|e\|}\hat{t}$ 

Where $\hat{t} = \begin{bmatrix} \cos(\psi) \\ \sin(\psi) \end{bmatrix}$ is the unit vector in the direction of travel.
Let $\hat{e} = \frac{e}{\|e\|}$ be the unit vector pointing from the thermal core to the glider (i.e. direction in which error grows).
Therefore, $\dot{V} = V r \frac{\hat{e}}{\hat{t}}$.

The angle between $\hat{t}$ and $\hat{e}$ is $\theta$, where $\cos(\theta) = \hat{e}^T \hat{t}$. We can see from this relationship that the Lyapunov function will decrease or remain asymptotically stable when $\cos(\theta) <= 0$, i.e. when the glider is maintaining a heading directly tangent to or towards the thermal core. This checks out intuitively, where the glider is either circling the thermal core or getting closer to the core. "Stability" in this case means that the glider is not getting further away from the core, which is what we're looking for in this analysis. 

## Controlling via Curvature
Remembering $\kappa = \frac{d\psi}{ds}$ and $\frac{ds}{dt} = V$, we can write $\kappa = \frac{\dot{\psi}}{\dot{s}}$. The curvature $\kappa$ is defined as the rate of change of heading angle $\psi$ with respect to distance traveled $s$. When subbing in $\dot{psi}$ from the glider model for coordinated turns, we get
$$
\kappa = \frac{g}{V^2} \tan(\phi)
$$
Solving for $\phi$ gives
$$
\phi = arctan(\frac{\kappa V^2}{g})
$$

Ideally, we expect that the curvature $\kappa$ will settle to a constant value as the glider centers in the thermal