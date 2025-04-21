import casadi as ca
import numpy as np
from DIPC.config import DIPCParams


def dynamics(x, u, params):
    """
    Compute x_dot = f(x,u) for the DIPC underactuated swing-up
    State x = [x, theta2, theta3, x_dot, theta2_dot, theta3_dot]
    Control u = [F_cart]
    """
    # unpack parameters
    Mc, M1, M2 = params.Mc, params.M1, params.M2
    L1, L2     = params.L1, params.L2
    g          = params.g
    I1, I2     = params.I1, params.I2

    # unpack state
    x1, th2, th3, v1, v2, v3 = x[0], x[1], x[2], x[3], x[4], x[5]

    # inertia matrix entries
    h1 = Mc + M1 + M2
    h2 = M1*(L1/2) + M2*L1
    h3 = M2*(L2/2)
    h4 = M1*(L1/2)**2 + I1 + M2*L1**2
    h5 = M2*(L2/2)*L1
    h6 = M2*(L2/2)**2 + I2

    # mass matrix (3x3)
    Mmat = ca.vertcat(
        ca.horzcat(h1,     h2*ca.cos(th2),      h3*ca.cos(th3)),
        ca.horzcat(h2*ca.cos(th2), h4,              h5*ca.cos(th2 - th3)),
        ca.horzcat(h3*ca.cos(th3), h5*ca.cos(th2 - th3), h6)
    )

    # Coriolis / centrifugal (3x1)
    C1 = -h2*v2*ca.sin(th2) - h3*v3*ca.sin(th3)
    C2 = -h5*(v2 - v3)*ca.sin(th2 - th3)
    C3 =  h5*(v2 - v3)*ca.sin(th2 - th3)
    Cvec = ca.vertcat(C1, C2, C3)

    # gravity (3x1)
    G1 = 0
    G2 = -h2*g*ca.sin(th2)
    G3 = -h3*g*ca.sin(th3)
    Gvec = ca.vertcat(G1, G2, G3)

    # input mapping (3x1)
    Bvec = ca.vertcat(u, 0, 0)

    # solve for accelerations [xdd, theta2dd, theta3dd]
    accel = ca.mtimes(ca.inv(Mmat), Bvec - Cvec - Gvec)

    # state derivative
    xdot = ca.vertcat(
        v1,
        v2,
        v3,
        accel[0],
        accel[1],
        accel[2]
    )
    return xdot


def solve_swingup(params, x0, xf, N=50, T_guess=2.0):
    """
    Formulate and solve a direct-collocation OCP in CasADi:
      min_u  \int_0^T u^2 dt
      s.t. x_dot = dynamics(x,u),  x(0)=x0, x(T)=xf
    Returns time grid, optimal X, optimal U
    """
    nx = 6
    nu = 1
    opti = ca.Opti()

    # decision variables
    X = opti.variable(nx, N+1)
    U = opti.variable(nu, N)
    T = opti.variable()       # final time

    # time step
    dt = T/N

    # boundary conditions
    opti.subject_to(X[:, 0] == x0)
    opti.subject_to(X[:, N] == xf)

    # dynamic constraints (Euler collocation)
    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]
        x_next = X[:, k+1]
        f_k = dynamics(xk, uk, params)
        opti.subject_to(x_next == xk + dt * f_k)

    # input & time bounds
    opti.subject_to(opti.bounded(-10, U, 10))
    opti.subject_to(T >= 0.1)
    opti.subject_to(T <= 10)

    # objective: minimize input effort and optionally time
    obj = ca.sumsqr(U) * dt
    opti.minimize(obj)

    # solver settings
    p_opts = {"expand": True}
    s_opts = {"max_iter": 1000, "tol":1e-6}
    opti.solver("ipopt", p_opts, {**s_opts, "linear_solver": "csparse"})

    # initial guesses
    opti.set_initial(T, T_guess)
    for k in range(N+1):
        alpha = k/N
        opti.set_initial(X[:,k], (1-alpha)*x0 + alpha*xf)
        if k < N:
            opti.set_initial(U[:,k], 0)

    sol = opti.solve()

    T_opt = sol.value(T)
    tgrid = np.linspace(0, T_opt, N+1)
    X_opt = sol.value(X)
    U_opt = sol.value(U)
    return tgrid, X_opt, U_opt


if __name__ == "__main__":
    # parameters
    params = DIPCParams()
    # start: inverted downward (angles = 0), end: upright (angles = pi/2)
    x0 = np.array([0, 0, 0, 0, 0, 0])
    xf = np.array([0, np.pi/2, 0, 0, 0, 0])

    t, X_opt, U_opt = solve_swingup(params, x0, xf)

    # quick plotting
    import matplotlib.pyplot as plt
    plt.figure(); plt.plot(t, X_opt[1,:], label='theta2')
    plt.plot(t, X_opt[2,:], label='theta3')
    plt.legend(); plt.show()
