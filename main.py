from DIPC.model import DIPC
from DIPC.config import DIPCParams
from DIPC.plot import plot_states, plot_control, animate
import numpy as np
import matplotlib.pyplot as plt

# 1. Create parameter object
params = DIPCParams()

# 2. Instantiate the model with parameters
model = DIPC(params)

# 3. Define initial state: [q1, q2, q3, dq1, dq2, dq3]
x0 = [0, np.pi/2.01, 0, 0.0, 0.0, 0.0]

# 4. Define a dummy control input (no input)
def u_func(x, t):
    return np.zeros(3)

# # 5. Define a dummy simulation function (since you haven't implemented model.simulate yet)
# # For now, just test lambdified functions
# q = x0[:3]
# dq = x0[3:]
# ddq = np.array([0.0, 0.0, 0.0])
# tau = model.force(q, dq, ddq)

# print("Generalized forces (tau):", tau)


# after simulating:
t, x = model.simulate(x0, u_func, t_final=5)

plot_states(t, x)
# plot_control(t, u_traj)  # if you're logging control inputs
animate(x, t)


