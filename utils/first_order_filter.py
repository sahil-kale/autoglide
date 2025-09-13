import numpy as np


def first_order_filter(x_prev, x_target, tau, dt):
    """
    First order low-pass filter.

    Args:
        x_prev: Previous filtered value.
        x_target: Target value to filter towards.
        tau: Time constant of the filter (s).
        dt: Time step (s).

    Returns:
        New filtered value.
    """
    alpha = dt / (tau + dt)
    return (1 - alpha) * x_prev + alpha * x_target
