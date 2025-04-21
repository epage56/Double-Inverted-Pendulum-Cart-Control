# from DIPC.model import DIPC
# from DIPC.config import DIPCParams
# from DIPC.plot import plot_states, plot_control, animate
# import numpy as np
# import matplotlib.pyplot as plt

# # 1. Create parameter object
# params = DIPCParams()

# # 2. Instantiate the model with parameters
# model = DIPC(params)

# # 3. Define initial state: [q1, q2, q3, dq1, dq2, dq3]
# x0 = [0, -np.pi/2, 0, 0.0, 0.0, 0.0]

# # 4. Define a dummy control input (no input)
# def u_func(x, t):
#     return np.zeros(3)

# # after simulating:
# t, x = model.simulate(x0, u_func, t_final=5)

# plot_states(t, x)
# # plot_control(t, u_traj)  # if you're logging control inputs
# animate(x, t)


import numpy as np
from DIPC.model   import DIPC
from DIPC.config  import DIPCParams
from DIPC.control import get_lqr, make_u_lqr
from DIPC.plot    import plot_states, plot_control, animate

# 1) Instantiate your model
params = DIPCParams()
model  = DIPC(params)

# 2) Define the upright equilibrium
x_eq = [0, np.pi/2, 0, 0, 0, 0]   # [x, θ2, θ3, dx, dθ2, dθ3] = all zero → upright
u_eq = np.zeros(3)    # zero input

# 3) Linearize about that point
A, B = model.linearize(x_eq, u_eq)

# 4) Choose LQR weights
Q = np.diag([100, 100, 100, 1, 1, 1])   # penalize angles heavily
R = np.eye(3) * 0.1                    # small control cost

# 5) Compute the gain
K = get_lqr(A, B, Q=Q, R=R)

# 6) Build your feedback law
u_func = make_u_lqr(K, x_eq=x_eq, u_eq=u_eq)

# 7) Simulate a small perturbation
x0 = x_eq.copy()
x0[1] += 0.5   # a 0.1 rad kick to link 1
t, x_traj = model.simulate(x0, u_func, t_final=5.0, dt=0.01)

# 8) Build the control trajectory for plotting
u_traj = np.vstack([u_func(x_traj[i], t[i]) for i in range(len(t))])

# 9) Plot results
plot_states(t, x_traj)
plot_control(t, u_traj)
animate(x_traj, t)

