import sympy as sym
import numpy as np
from scipy.integrate import solve_ivp

class DIPC:
    def __init__(self, params):
        """
        Initialize the system with the given parameters.
        """
        self.params = params
        self._define_symbols()
        self._derive_eom()
        self._lambdify_rhs()
        
    def _define_symbols(self):
        """
        Define symbolic variables for generalized coordinates and inputs.
        """
        self.t = sym.symbols("t")
        self.q = sym.Array(
            [sym.Function("x1")(self.t), sym.Function("θ2")(self.t), sym.Function("θ3")(self.t)]
            )
        self.dq = self.q.diff(self.t)
    
    def _derive_eom(self):
        """
        Derive equations of motion using the Lagrangian method.
        """
        Mc = self.params.Mc
        M1 = self.params.M1
        M2 = self.params.M2
        L1 = self.params.L1
        L2 = self.params.L1 
        G = self.params.g
        G1 = self.params.G1
        G2 = self.params.G2
        G3 = self.params.G3
        
        
          # link poses
        P1 = sym.Matrix([self.q[0], 0, 0])
        P2 = P1 + sym.Matrix([
            0.5 * L1 * sym.cos(self.q[1]),
            0.5 * L1 * sym.sin(self.q[1]),
            self.q[1],
        ])
        P3 = P2 + sym.Matrix([
            0.5 * L1 * sym.cos(self.q[1]) + 0.5 * L2 * sym.cos(self.q[1] + self.q[2]),
            0.5 * L1 * sym.sin(self.q[1]) + 0.5 * L2 * sym.sin(self.q[1] + self.q[2]),
            self.q[2],
        ])

        # link Jacobians
        J1 = P1.jacobian(self.q)
        J2 = P2.jacobian(self.q)
        J3 = P3.jacobian(self.q)

        # mass matrix
        M = J1.transpose() * G1 * J1 + J2.transpose() * G2 * J2 + J3.transpose() * G3 * J3

        # Christoffel symbols and Coriolis matrix
        dMdq = M.diff(self.q)
        Γ = sym.permutedims(dMdq, (2, 1, 0)) - 0.5 * dMdq
        C = sym.tensorcontraction(sym.tensorproduct(self.dq, Γ), (0, 2)).tomatrix()

        # gravity vector
        V = G * (Mc * P1[1] + M1 * P2[1] + M2 * P3[1])
        g = V.diff(self.q)
        
        self.M = M
        self.C = C
        self.g = g
        
    def _lambdify_rhs(self):
        """
        Lambdify symbolic functions to numerical ones for fast evaluation.
        """
        self.mass_matrix = sym.lambdify([self.q], self.M, modules="numpy")
        self.coriolis_matrix = sym.lambdify([self.q, self.dq], self.C, modules="numpy")
        self.gravity_vector = sym.lambdify([self.q], self.g, modules="numpy")
    
    def force(self, q, dq, ddq):
        """
        Compute the generalized forces given state and acceleration.

        Parameters:
            q: configuration vector (shape [3])
            dq: velocity vector (shape [3])
            ddq: acceleration vector (shape [3])

        Returns:
            tau: generalized forces (shape [3])
        """
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, dq)
        g = self.gravity_vector(q)

        return M @ ddq + C @ dq + g

    def simulate(self, x0, u_func, t_final, dt=0.01):
        """
        Simulate the system using forward integration.
        """
        def ode(t, state):
            q = state[0:3]
            dq = state[3:6]

            # Here you compute ddq from M, C, g, and u
            u = u_func(state, t)
            M = self.mass_matrix(q)
            C = self.coriolis_matrix(q, dq)
            g = self.gravity_vector(q)

            ddq = np.linalg.solve(M, u - C @ dq - g)
            return np.concatenate([dq, ddq])

        t_eval = np.arange(0, t_final + dt, dt)
        sol = solve_ivp(ode, (0, t_final), x0, t_eval=t_eval, method='RK45')

        if not sol.success:
            raise RuntimeError("Simulation failed")

        return sol.t, sol.y.T

    def linearize(self, x_eq, u_eq):
        """
        Linearize the system at the given equilibrium point.
        """
        pass  # TODO: implement