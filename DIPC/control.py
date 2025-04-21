# control.py

import numpy as np
import control  # pip install control

def get_lqr(A, B, Q=None, R=None):
    """
    Continuous‑time LQR:
      minimize ∫(xᵀQx + uᵀRu) dt
    Solves CARE: AᵀP + PA − P B R⁻¹ Bᵀ P + Q = 0
    Returns gain K s.t. u = −K x.
    """
    n, m = A.shape[0], B.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)

    # control.lqr returns (K, S, E) where S = solution P, E = closed‑loop eigs
    K, P, eigs = control.lqr(A, B, Q, R)
    return np.array(K)

def get_dlqr(A, B, Q=None, R=None, dt=0.01):
    """
    Discrete‑time LQR:
      x[k+1] = Ad x[k] + Bd u[k]
    minimize Σ (xᵀQx + uᵀRu)
    Solves DARE: P = Adᵀ P Ad − Adᵀ P Bd (R + Bdᵀ P Bd)⁻¹ Bdᵀ P Ad + Q
    Returns K s.t. u[k] = −K x[k].

    dt: sampling time for discretization
    """
    n, m = A.shape[0], B.shape[1]
    Q = Q if Q is not None else np.eye(n)
    R = R if R is not None else np.eye(m)

    # build continuous‑time system and discretize
    sysc = control.ss(A, B, np.eye(n), np.zeros((n, m)))
    sysd = control.c2d(sysc, dt)
    Ad, Bd = sysd.A, sysd.B

    K, P, eigs = control.dlqr(Ad, Bd, Q, R)
    return np.array(K)

def make_u_lqr(K, x_eq=None, u_eq=None):
    """
    Build a state-feedback controller function:
       u(x, t) = -K (x - x_eq) + u_eq
    K: shape (m, n)
    x_eq: length-n equilibrium state
    u_eq: length-m equilibrium input
    """
    n = K.shape[1]
    m = K.shape[0]
    x_eq = np.zeros(n) if x_eq is None else np.asarray(x_eq).reshape(n,)
    u_eq = np.zeros(m) if u_eq is None else np.asarray(u_eq).reshape(m,)

    assert x_eq.shape == (n,)
    assert u_eq.shape == (m,)

    def u(x, t=None):
        x = np.asarray(x).reshape(n,)
        return -K.dot(x - x_eq) + u_eq

    return u