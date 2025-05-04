from DIPC.model import DIPC
from DIPC.config import DIPCParams
from DIPC.plot import plot_states, animate_dipc
import numpy as np

# Create params object
params = DIPCParams()

# Init the model with params
model = DIPC(params)

# make the initial state of the device something not quite at inverted [q1, q2, q3, dq1, dq2, dq3]
x0 = [0, np.pi/2.1, 0, 0.0, 0.0, 0.0]

# Force define the control input to be 0 (no forcing)
def u_func(x, t):
    return np.array([0.0])

t, x = model.simulate(x0, u_func, t_final=10)

# plot the states of the model evolving over time to observe them
plot_states(t, x)

# plot the animattion of the system 
animate_dipc(x, t)