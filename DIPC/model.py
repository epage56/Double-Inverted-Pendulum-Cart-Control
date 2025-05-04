import numpy as np
import casadi as ca
from DIPC.config import DIPCParams
from scipy.integrate import solve_ivp

class DIPC:
    def __init__(self, params: DIPCParams):
        self.params = params
        self._build_symbolic_dynamics()

    def _build_symbolic_dynamics(self):
        
        # use the casadi sx symbol obj to define all the symbols used in the dynamics
        q1, q2, q3 = ca.SX.sym('q1'), ca.SX.sym('q2'), ca.SX.sym('q3')
        dq1, dq2, dq3 = ca.SX.sym('dq1'), ca.SX.sym('dq2'), ca.SX.sym('dq3')
        u1 = ca.SX.sym('u1')

        # make 'em vectors for all the matrix math
        q  = ca.vertcat(q1, q2, q3)
        dq = ca.vertcat(dq1, dq2, dq3)
        x  = ca.vertcat(q, dq)
        u  = ca.vertcat(u1)

        # calling params to get all the physical parameters.
        
        P = self.params
        Mc, M1, M2 = P.Mc, P.M1, P.M2
        L1, L2     = P.L1, P.L2
        g          = P.g
        Bc, B1, B2 = P.Bc, P.B1, P.B2
        I1, I2     = P.I1, P.I2

        # define inertia vectoprs
        G1 = ca.diag(ca.vertcat(Mc, Mc, 0))
        G2 = ca.diag(ca.vertcat(M1, M1, I1))
        G3 = ca.diag(ca.vertcat(M2, M2, I2))

        # getting the position of the center of mass of each body 
        P1 = ca.vertcat(q1, 0, 0)
        P2 = P1 + ca.vertcat(0.5*L1*ca.cos(q2), 0.5*L1*ca.sin(q2), q2)
        P3 = P2 + ca.vertcat(
            0.5*L1*ca.cos(q2) + 0.5*L2*ca.cos(q2+q3),
            0.5*L1*ca.sin(q2) + 0.5*L2*ca.sin(q2+q3),
            q3
        )

        # Computing the jacobian of those positions to make the mass matrix and 
        J1 = ca.jacobian(P1, q)
        J2 = ca.jacobian(P2, q)
        J3 = ca.jacobian(P3, q)

        # assemble the mass or inertia matrix, M
        M_mat = J1.T @ G1 @ J1 + J2.T @ G2 @ J2 + J3.T @ G3 @ J3

        # This is disgusting, assembling the corilis matrix by christoffel symbols
        C_mat = ca.SX.zeros(3, 3)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    # partial derivatives via jacobian
                    dMij = ca.jacobian(ca.vertcat(M_mat[i, j]), q)
                    dMik = ca.jacobian(ca.vertcat(M_mat[i, k]), q)
                    dMjk = ca.jacobian(ca.vertcat(M_mat[j, k]), q)
                    dMij_dqk = dMij[0, k]
                    dMik_dqj = dMik[0, j]
                    dMjk_dqi = dMjk[0, i]
                    Gamma = 0.5 * (dMij_dqk + dMik_dqj - dMjk_dqi)
                    C_mat[i, j] += Gamma * dq[k]

        # define the gravity vector
        V = g * (Mc*P1[1] + M1*P2[1] + M2*P3[1])
        g_vec = ca.gradient(V, q)

        # defining friction at the joints with viscous friction model
        friction = ca.vertcat(Bc*dq1, B1*dq2, B2*dq3)

        # finally, construct the dynamics and solve them
        tau = ca.vertcat(u1, 0, 0)
        ddq = ca.solve(M_mat, tau - C_mat@dq - g_vec - friction)

        # here is xdot
        xdot = ca.vertcat(dq, ddq)

        # make f an actual casadi function
        self.f = ca.Function('f', [x, u], [xdot], ['x','u'], ['xdot'])

    def simulate(self, x0, u_func, t_final, dt=0.01):
        """
        Solves the dynamics equations with rk4 integrator forward
        
        INPUTS:
            x0: initial state vector
            u_func: control input force; function(state, t) -> length-1 array
        """
        
        # make an ode function to be solved with solve_ivp from scipy integrate
        def ode(t, y):
            u = u_func(y, t)
            dx = np.array(self.f(y, u)).flatten()
            return dx

        # make time array 
        t_eval = np.arange(0, t_final + dt, dt)
        # solve the initial value problem and allow it to evolve forward in time forever 
        sol = solve_ivp(ode, [0, t_final], x0, t_eval=t_eval, method='RK45')

        # return the solution time array and states 
        return sol.t, sol.y.T

    def linearize(self, x_eq, u_eq):
        """
        Linearize the dynamics around the inverted equilia
        
        INPUTS:
            x_eq: the equilibria position to be linearized about
            u_eq: the equilibria control input to be linearized about
            
        OUTPUTS:
            A:  state matrix of the linearized system
            B:  input matrix of the linearized system
        """
        
        x_sym = ca.SX.sym('x', 6) # make a state vector in symbolic form
        u_sym = ca.SX.sym('u', 1) # make the input vector 
        
        f_sym = self.f(x_sym, u_sym) # eval the dynamics function
        
        # computes the jacobians to construct the A and B matrices for linearization
        A_sym = ca.jacobian(f_sym, x_sym)
        B_sym = ca.jacobian(f_sym, u_sym)
        
        # make them callable functions with the casadi lib, evals them
        # retunrs ndarrays (6x6 and 6x1)
        A = ca.Function('A', [x_sym, u_sym], [A_sym])(x_eq, u_eq).full()
        B = ca.Function('B', [x_sym, u_sym], [B_sym])(x_eq, u_eq).full()
        
        # store them for later use
        self.A, self.B = A, B
        return A, B
