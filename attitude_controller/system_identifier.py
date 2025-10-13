import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vehicle_interface import vehicle_interface
from jsbsim_sandbox.sandbox_sim import *
import numpy as np
import jsbsim

class SystemIdentifierSingleCase:
    def __init__(self, sim: JSBSim_Sandbox, initial_cond: JSBSimVehicleInitialCond):
        self.sim = sim
        self.initial_cond = initial_cond

    def run(self, )
