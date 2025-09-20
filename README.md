# AutoGlide
## Overview
Autoglide is a short side project I started to learn about glider flight dynamics, thermal updraft modelling, state estimation, and non-linear guidance controls. The goal is to create a simulation of a glider within a convective environment that can autonomously locate and track a thermal updraft using only onboard sensors and compute, mirroring the challenges faced by real-world glider pilots. The project ultimately ended up being a fun way to explore non-linear control theory, optimal control, state estimation, and controller/estimator interactions.

The project was inspired by my interest in gliding as a sport (as I am a glider pilot myself) and my desire to learn more about control theory and state estimation.

![Glider Simulation](circling_demo.gif)

## Table of Contents
- [Glider Model](glider_model/readme.md)
- [Thermal Model](thermal_model/readme.md)
- [Core Tracking Filter + State Estimator](thermal_estimator/readme.md)
- [Guidance Control Laws](controller/readme.md)
- [Monte Carlo Simulation + Analysis](monte_carlo/readme.md)

## Future Work
Here are some random ideas and thoughts for extensions and future work that I wanted to capture.
- Windshear thermal models
- Condor 2 Integration (Plug and Play with XCSoar UDP API, Virtual Joystick)
- Variometer modelling
- Wind estimation
- Replay real thermal data and see how well it would do

## Asides
This is a side project for the sake of learning and fun. It is not intended to be a project with production quality code or an example of best practice engineering - it's meant to be a sandbox for me to learn and experiment. AI was used to help generate some of the code and understand concepts presented in various research papers. If you find this project useful or interesting, please consider starring the repository!

## Intuition
Overwhelmingly while developing this project, I found that my intuition about the control and estimation problems proved to be very valuable. For example, I had a strong intuition that the glider should circle in a thermal to stay within the updraft, and that the circling radius should be chosen based on a tradeoff between sink rate and updraft strength, and ultimately required an online optimization to solve. 

It was also really interesting to see the physics-based model of the glider and thermal updrafts come together to form a coherent simulation that behaved in a realistic manner. I found that having a strong understanding of the underlying physics and mathematics was crucial to being able to implement the models and algorithms correctly. 

Funnily enough, the controller behaves very similarly to how I would intuitively try to fly a glider in real life, which was a nice validation of the approach. For instance, the controller that determines the optimal circling radius and speed-to-fly in the majority of cases elects to fly at or near stall speed to get the best climb rate. This is exactly how I maximize thermal climb rate when I fly gliders in real life - fly slow, tight circles on the verge of a stall to maximize the time spent in the strongest part of the thermal.