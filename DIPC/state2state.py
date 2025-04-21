def state2state(model, x0, xf, T=5.0, N=100):
    """
    Optimize a control trajectory that moves the system from x0 to xf.

    Parameters:
        model: DIPC instance with rhs_func available
        x0: initial state vector (length 6)
        xf: final state vector (length 6)
        T: time horizon in seconds
        N: number of discretization steps

    Returns:
        t: time vector
        x_traj: optimal state trajectory (N x 6)
        u_traj: optimal control trajectory (N x 3)
    """
    pass