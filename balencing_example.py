import numpy as np

from DIPC.config import DIPCParams
from DIPC.model import DIPC
from DIPC.control import get_lqr, make_u_lqr
from DIPC.plot import plot_states, plot_control, animate_dipc

# Build model and equilibrium
params = DIPCParams()
model  = DIPC(params)

# Equilibrium: cart at zero, first link up (π/2), second link hanging straight (0)
x_eq = np.array([0.0, np.pi/2, 0.0, 0.0, 0.0, 0.0])
u_eq = np.array([0.0])

# 2) Linearize and design LQR
A, B = model.linearize(x_eq, u_eq)
Q_lqr = np.diag([100, 100, 100,   1,   1,   1])   # heavy penalty on angle errors
R_lqr = np.array([[1.0]])                        # moderate control penalty
K = get_lqr(A, B, Q=Q_lqr, R=R_lqr)
u_lqr = make_u_lqr(K, x_eq=x_eq, u_eq=u_eq)

# Simulate closed-loop via a small perturbation initially 
x0 = x_eq + np.array([ 0.1, 0.1, -0.1, 0, 0, 0 ])
t_final = 5.0
dt = 0.01

t, x_traj = model.simulate(x0, u_lqr, t_final, dt=dt)

# Extract the scalar control at each step
u_traj = np.array([u_lqr(xi) for xi in x_traj]).reshape(-1, 1)

# Plot everyhting
plot_states(t, x_traj)   # each of the six states over time
plot_control(t, u_traj)  # control effort vs time
ani = animate_dipc(x_traj, t=t, # animation
                   L1=params.L1, 
                   L2=params.L2)