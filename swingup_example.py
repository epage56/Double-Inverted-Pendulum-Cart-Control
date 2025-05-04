import numpy as np

from DIPC.config import DIPCParams
from DIPC.model import DIPC
from DIPC.trajectory_opt import solve_swingup_ocp
from DIPC.plot import plot_states, plot_control, animate_dipc

# Model setup
params  = DIPCParams()
x0 = np.array([-0.5, -np.pi/2, 0, 0, 0, 0])
xf = np.array([ 0.0,  np.pi/2, 0, 0, 0, 0])
t_final = 2.5
N = 300

# Solve swing up
t_grid, u_swing = solve_swingup_ocp(
    params, x0, xf, t_final, N=N,
    R = np.array([[0.01]])
)

# simulate the model with that input
model = DIPC(params)
t_sim, x_sim = model.simulate(
    x0,
    lambda x, t: float(
        np.interp(
            t,
            t_grid[:-1],         # now length N
            u_swing[:, 0],        # also length N
            left=0.0,
            right=0.0
        )
    ),
    t_final,
    dt=t_final/(N-1)
)

# Plot the states & control input to see what they look like
plot_states(t_sim, x_sim)
plot_control(t_grid[:-1], u_swing)

# Animate the result to verify physicality 
ani = animate_dipc(x_sim, t=t_sim, L1=params.L1, L2=params.L2)