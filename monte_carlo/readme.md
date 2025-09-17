# Monte Carlo Simulations
This directory contains scripts and configurations for running Monte Carlo simulations of the glider model under various conditions.

## Why Monte Carlo Simulations?
In control theory, Monte Carlo simulations are used to assess the performance and robustness of control algorithms by running multiple simulations with varying initial conditions and parameters. We want to essentially stress-test our algorithms to ensure they can handle a wide range of scenarios, and expose any potential weaknesses or failure modes where we lack robustness.

## Simulations Run
The main types of simulations run include:

**Thermal Center Offset**: Varying the initial position of the glider relative to the thermal center. This tests the state estimator's ability to locate the thermal and the guidance algorithm's ability to center in the thermal.
- The primary goal of this is to evaluate the predicted thermal strength, center, radius, and ultimately the steady-state uplift the glider can achieve.

**Wind Conditions**: Introducing different wind speeds and directions to evaluate how well the glider can maintain its course and altitude while centering thermals.
- The primary goal of this is to evaluate the average distance from the thermal center the glider can achieve, as well as the steady-state average uplift.
