import numpy as np
import casadi as ca
from DIPC.config import DIPCParams
from DIPC.model import DIPC

def solve_swingup_ocp(params: DIPCParams,
                      x0: np.ndarray,
                      xf: np.ndarray,
                      t_final: float,
                      N: int = 50,
                      Q: np.ndarray = None,
                      R: np.ndarray = None):
    """
    Solve an the trajectory optimization problem via direct transcitption using CasADi.
    
    INPUTS:
        params : DIPCParams
            Physical and simulation parameters for the inverted pendulum system.
        x0 : ndarray, shape (6,)
            Initial state vector [x, θ1, θ2, dx, dθ1, dθ2].
        xf : ndarray, shape (6,)
            Desired final (equilibrium) state.
        t_final : float
            Total time horizon for the maneuver (seconds).
        N : int, optional
            Number of discrete intervals (default 50).
        Q : ndarray, shape (6,6), optional
            State weighting matrix for the cost. Defaults to identity.
        R : ndarray, shape (1,1), optional
            Control weighting matrix for the cost. Defaults to identity.
    
    OUTPUS:
        t_grid: time vecotr
        u_seq: force input at each time point
    """
    
    # init the system and pull the dynamics function
    model = DIPC(params)
    f = model.f

    # define the dims that we expect 
    nx, nu = 6, 1
    dt = t_final / N
    
    # make the weighting matrices just identity if the user does not input anything
    Q = Q if Q is not None else np.eye(nx)
    R = R if R is not None else np.eye(nu)

    opti = ca.Opti()
    X    = opti.variable(nx, N+1)
    U    = opti.variable(nu, N)

    # define the boundary conditions
    opti.subject_to(X[:, 0] == x0) # init state 
    opti.subject_to(X[:, -1] == xf) # final state
    opti.subject_to(opti.bounded(-50, U, 50)) # input limiting as the device cannot output this much force

    # warm start the search by guessing something
    opti.set_initial(X, np.tile(x0.reshape(-1,1), (1, N+1)))
    opti.set_initial(U, np.zeros((1, N)))

    # for loop that does r4k colocation (standard thing frn class)
    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]
        k1 = f(xk, uk)
        k2 = f(xk + dt/2*k1, uk)
        k3 = f(xk + dt/2*k2, uk)
        k4 = f(xk + dt*k3, uk)
        x_next = xk + dt/6*(k1 + 2*k2 + 2*k3 + k4)
        opti.subject_to(X[:, k+1] == x_next)

    # OBJECTIVE HERE: state deviation + control effort 
    # the reason I am using control effort (integral of force) is that
    # i want a smooth control input and not a bangbang style control
    J = 0
    for k in range(N):
        dx = X[:,k] - xf
        J += ca.mtimes([dx.T, Q, dx]) + ca.mtimes([U[:,k].T, R, U[:,k]])
    opti.minimize(J)

    # set up the solver with basic limits
    opti.solver('ipopt', {
        'print_time':        False,
        'ipopt.max_iter':    1000,
        'ipopt.tol':         1e-3,
        'ipopt.constr_viol_tol': 1e-3
    })
    
    # solve it (fingers crossed)
    sol = opti.solve()

    # make the time grid the correct size
    t_grid = np.linspace(0, t_final, N+1)
    # output the control input correctly
    u_seq  = sol.value(U).reshape(N, nu)
    return t_grid, u_seq