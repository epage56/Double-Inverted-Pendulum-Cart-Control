import numpy as np
from scipy.linalg import solve_continuous_are

def get_LQR(A, B, Q, R):
    """
    Compute the optimal gain matrix K using LQR.
    """
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K

def u_lqr(K):
    """
    Return a control function u(x, t) = -K @ x
    """
    return lambda x, t: -K @ x