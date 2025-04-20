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
    """
    Animate the cart and double inverted pendulum.
    
    Assumes:
        - x_traj[:, 0] = cart position (x)
        - x_traj[:, 1] = θ1 (first link)
        - x_traj[:, 2] = θ2 (second link)
    """
    x = x_traj[:, 0]
    θ1 = x_traj[:, 1]
    θ2 = x_traj[:, 2]
    
    L1 = 1.0  # assumed link length
    L2 = 0.5

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(-2, 2)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect("equal")

    cart_line, = ax.plot([], [], 'k', lw=6)
    link1_line, = ax.plot([], [], 'r-', lw=2)
    link2_line, = ax.plot([], [], 'b-', lw=2)

    def update(i):
        cart_x = x[i]

        # Joint positions
        joint1 = (cart_x, 0)
        joint2 = (cart_x + L1 * np.sin(θ1[i]),
                  -L1 * np.cos(θ1[i]))
        joint3 = (joint2[0] + L2 * np.sin(θ1[i] + θ2[i]),
                  joint2[1] - L2 * np.cos(θ1[i] + θ2[i]))

        cart_line.set_data([cart_x - 0.2, cart_x + 0.2], [0, 0])
        link1_line.set_data([joint1[0], joint2[0]], [joint1[1], joint2[1]])
        link2_line.set_data([joint2[0], joint3[0]], [joint2[1], joint3[1]])
        return cart_line, link1_line, link2_line

    ani = animation.FuncAnimation(fig, update, frames=len(x), interval=30, blit=True)
    plt.show()