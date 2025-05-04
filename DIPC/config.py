#config.py
from dataclasses import dataclass
import numpy as np

@dataclass
class DIPCParams:
    """
    Class to make storing the physical parameters for the system easy
    """
    # Mass of each body
    Mc: float = 3.0
    M1: float = 2.0
    M2: float = 1.0

    # Length of the pendulum(s)
    L1: float = 0.5
    L2: float = 0.5
    
    # Damping coefficient for viscous friction implementation per joint
    Bc: float = 0.2
    B1: float = 0.05
    B2: float = 0.05
    
    # Gravity
    g: float = 9.81

    # Moments of inertia 
    @property
    def I1(self):
        return self.M1 * self.L1**2 / 12

    @property
    def I2(self):
        return self.M2 * self.L2**2 / 12