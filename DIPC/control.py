# control.py

import numpy as np
import control # like the actual control library that is popular

def get_lqr(A, B, Q=None, R=None):
    """
    Returns the gain matrix, k such that u = -K x 
    by solving the continuous algebraic ricatti equation.
    
    INPUTS:
      A: ndarray, state transition matrix
      B: ndarray, control input matrix
      Q: ndarray,state weighting matrix
      R: ndarray, control weighting matrix
      
    OUPUTS:
      K: ndarray, optimal feedback gain matrix

    
    """

    # control.lqr returns (K, S, E) : S = solution P, E = closed‑loop eigs
    # we want the gain matrix
    K, P, eigs = control.lqr(A, B, Q, R)
    
    return np.array(K)

def make_u_lqr(K, x_eq=None, u_eq=None):
    """
    Method that actually builds the state-feedback controller:
       u(x, t) = -K (x - x_eq) + u_eq
       
    INPUTS:
      K: ndarray, gain matrix (m,n) shape
      x_eq: n length eq state
      u_eq: m length eq input
      
    OUTPUTS: 
      u: control input array
    """
    
    # compute the control input for a given state 
    def u(x, t=None):
        return -K.dot(x - x_eq) + u_eq

    return u