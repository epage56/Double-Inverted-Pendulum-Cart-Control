
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

def plot_states(t, x_traj):
    """
    Plot each state variable over time.
    
    Parameters:
        t: time vector
        x_traj: state trajectory, shape [len(t), 6]
    """
    labels = ["x (cart position)", "θ1", "θ2", "dx", "dθ1", "dθ2"]
    
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))
    axs = axs.flatten()

    for i in range(6):
        axs[i].plot(t, x_traj[:, i])
        axs[i].set_title(labels[i])
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel("Value")
        axs[i].grid(True)

    plt.tight_layout()
    plt.show()

def plot_control(t, u_traj):
    """
    Plot control inputs over time.
    
    Parameters:
        t: time vector
        u_traj: control inputs, shape [len(t), 1] or [len(t), num_controls]
    """
    u_traj = u_traj if u_traj.ndim > 1 else u_traj[:, np.newaxis]
    num_controls = u_traj.shape[1]

    plt.figure(figsize=(10, 4))
    for i in range(num_controls):
        plt.plot(t, u_traj[:, i], label=f"u{i+1}")
    
    plt.title("Control Inputs")
    plt.xlabel("Time (s)")
    plt.ylabel("Input Force / Torque")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

def animate(x_traj, t=None):
    # unpack states
    x   = x_traj[:, 0]   # cart horizontal position
    θ2  = x_traj[:, 1]   # first link angle (0 = horizontal right)
    θ3  = x_traj[:, 2]   # second link relative angle

    L1 = 0.5
    L2 = 0.5

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(-2,  2)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")

    cart_line,  = ax.plot([], [], 'k',  lw=6)
    link1_line, = ax.plot([], [], 'r-', lw=2)
    link2_line, = ax.plot([], [], 'b-', lw=2)

    def update(i):
        cx = x[i]

        # base of pendulum (cart axle)
        base = np.array([cx, 0.0])

        # world‑frame COM of link1
        p1 = base + np.array([
            L1 * np.cos(θ2[i]),
            L1 * np.sin(θ2[i])
        ])

        # world‑frame COM of link2
        p2 = p1 + np.array([
            L2 * np.cos(θ2[i] + θ3[i]),
            L2 * np.sin(θ2[i] + θ3[i])
        ])

        # draw the little “cart”
        cart_line.set_data([cx - .2, cx + .2], [0, 0])

        # draw each link
        link1_line.set_data([base[0], p1[0]], [base[1], p1[1]])
        link2_line.set_data([p1[0],  p2[0]], [p1[1],  p2[1]])

        return cart_line, link1_line, link2_line

    ani = animation.FuncAnimation(
        fig, update,
        frames=len(x),
        interval=10,
        blit=True
    )
    plt.show()