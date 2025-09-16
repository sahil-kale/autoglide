import sympy as sp
import numpy as np

# Define symbols for thermal parameters
w0, Rth, xc, yc = sp.symbols("w0 Rth xc yc")

ESTIMATED_PARAMS_SYMBOLS = [w0, Rth, xc, yc]

# Define symbols for aircraft position
x_i, y_i = sp.symbols("x_i y_i")
AIRCRAFT_POSITION_SYMBOLS = (x_i, y_i)

# Define symbol for measured vertical air velocity (variometer measurement)
w_meas_i = sp.symbols("w_meas_i")


def basic_thermal_model(aircraft_position, estimated_params):
    x, y = aircraft_position
    w0, Rth, xc, yc = estimated_params
    r = sp.sqrt((x - xc) ** 2 + (y - yc) ** 2)
    return w0 * sp.exp(
        -((r / Rth) ** 2)
    )  # Reduced order Gaussian model that the estimator uses


def jacobian_of_thermal_model():
    model = basic_thermal_model(AIRCRAFT_POSITION_SYMBOLS, ESTIMATED_PARAMS_SYMBOLS)
    jacobian = [sp.diff(model, param) for param in ESTIMATED_PARAMS_SYMBOLS]
    # simplify the expressions
    jacobian = [sp.simplify(expr) for expr in jacobian]
    return jacobian


jacobian = jacobian_of_thermal_model()
print("Jacobian symbols:", jacobian)
