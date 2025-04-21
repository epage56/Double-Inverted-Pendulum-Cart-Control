from dataclasses import dataclass
import numpy as np

@dataclass
class DIPCParams:
    """
    Stores physical parameters for the DIPC system.
    """
    # Body masses
    Mc: float = 3.0
    M1: float = 2.0
    M2: float = 1.0

    # Pendulum lengths
    L1: float = 0.5
    L2: float = 0.5
    
    # Coefficients of friciion:
    Bc: float = 0.2
    B1: float = 0.05
    B2: float = 0.05
    
    # Gravity
    g: float = 9.81

    # Moments of inertia (computed from mass and length)
    @property
    def I1(self):
        return self.M1 * self.L1**2 / 12

    @property
    def I2(self):
        return self.M2 * self.L2**2 / 12

    # Inertia matrices (used in Lagrangian modeling)
    @property
    def G1(self):
        return np.diag([self.Mc, self.Mc, 0.0])

    @property
    def G2(self):
        return np.diag([self.M1, self.M1, self.I1])

    @property
    def G3(self):
        return np.diag([self.M2, self.M2, self.I2])
