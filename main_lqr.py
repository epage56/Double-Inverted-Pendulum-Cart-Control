# main_lqr.py

import numpy as np
from DIPC.model   import DIPC
from DIPC.config  import DIPCParams
from DIPC.control import get_lqr
from DIPC.plot    import plot_states, plot_control, animate

def main():
    # 1) Instantiate your model
    params = DIPCParams()
    model  = DIPC(params)

    # 2) Define the upright equilibrium
    #    State: [x, θ2, θ3, dx, dθ2, dθ3]
    x_eq = [0, np.pi/2, 0, 0, 0, 0]
    #    Inputs: [F_cart, τ1, τ2]
    u_eq = np.zeros(3)

    # 3) Linearize about that point
    A, B = model.linearize(x_eq, u_eq)

    # 4) Keep only the cart‑force channel (underactuated)
    #    B is 6×3; column 0 is the cart
    B_cart = B[:, [0]]    # shape (6,1)

    # 5) Choose LQR weights
    Q = np.diag([100, 100, 100, 1, 1, 1])
    R = np.array([[1.0]])  # scalar penalty on cart force

    # 6) Compute the single‑input LQR gain
    K_cart = get_lqr(A, B_cart, Q=Q, R=R)   # shape (1,6)

    # 7) Build the underactuated feedback law
    def u_underact(x, t=None):
        x = np.asarray(x).reshape(6,)
        # u1 = -K_cart · (x − x_eq)
        u1 = -K_cart.dot(x - x_eq).item()
        # zero torques at the joints
        return np.array([u1, 0.0, 0.0])

    # 8) Simulate a small perturbation
    x0 = x_eq.copy()
    x0[1] += 0.3   # small angle kick on link 1
    t, x_traj = model.simulate(x0, u_underact, t_final=5.0, dt=0.01)

    # 9) Build the control trajectory for plotting
    u_traj = np.vstack([u_underact(x_traj[i], t[i]) for i in range(len(t))])

    # 10) Visualize
    plot_states(t, x_traj)
    plot_control(t, u_traj)
    animate(x_traj, t)

if __name__ == "__main__":
    main()