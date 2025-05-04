import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def plot_states(t, x_traj):
    """
    Plot each state variable over time positions, velocities etc.
    
    INPUTS:
        t: time vector
        x_traj: state vector over time, describing 
                evolution of system - shape [len(t), 6]
    OUPUTS:
        one figure of all states over time
    """
    
    # make some labels to make the plots readable (remmebr this x is not the state space x)
    labels = ["x (cart pos.)", "θ1", "θ2", "dx", "dθ1", "dθ2"]
    
    # make the fig, make 6 total plots
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))
    axs = axs.flatten()
    
    # plot everything we have in the trajectory array
    # x units are always time
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
    Plots the control input over time to be able to see the if the control input is 
    feasible or not. 
    
    INPUTS:
        t: time vector
        u_traj: control input value over time, describing 
                evolution of system
    """
    # make sure that the control input, u_traj is shaped (N, m) evejn if started as 2d or 1d
    if u_traj.ndim == 1:
        u_arr = u_traj[:, None]
    else:
        u_arr = u_traj

    # make a figure/axes
    fig, ax = plt.subplots(figsize=(8, 4))

    # Plot each column as u1, u2, u3. friction works on u2, u3
    for idx in range(u_arr.shape[1]):
        ax.plot(t, u_arr[:, idx], label=f'u{idx+1}')

    # Labeling + grid
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Force')
    ax.legend()
    ax.grid(True)

    plt.tight_layout()
    plt.show()
    
def animate_dipc(x_traj, t=None, L1=0.5, L2=0.5):
    """
    Animate a double inverted pendulum on a cart in 2D.

    INPIUTS:

        x_traj : State trajectory over N frames, ndarray: 
            cols:
                0: cart position
                1: first pendulum angle
                2: second pendulum angle
                3–5: velocities
                
        t : time vector, used to align fps of animate with sim fps so it animates in "real-time"
        L1: float, length of first link
        L2: float, length of the second link.


    OUTPUTS:
        ani: animated plot
    """

    x    = x_traj[:, 0]  # cart positions
    theta1 = x_traj[:, 1]  # link1 absolute angle
    theta2 = x_traj[:, 2]  # link2 relative angle

    # selkecting the playback speed
    if t is None:
        interval = 10  # ms / frame
    else:
        dt = np.diff(t)
        interval = float(dt.mean() * 1000)  # convert to ms = *1000

    # make fig and axes
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.set_xlim(-2, 2)
    ax.set_ylim(-1.5, 1.5)
    ax.set_aspect('equal')  # make sure that no distortion occurs

    # init the obj on the plot (cart as line, links as colored lines) 
    
    cart_line, = ax.plot([], [], color='k', linewidth=8)
    link1_line, = ax.plot([], [], color='r', linewidth=2)
    link2_line, = ax.plot([], [], color='b', linewidth=2)

    # define a function to update the frames of the plot with the new positions
    def update(frame_idx):
        # Current cart x-position
        cx = x[frame_idx]
        base_pt = np.array([cx, 0.0])

        # forward kinematics: get each joint position
        
        joint1 = base_pt + np.array([
            L1 * np.cos(theta1[frame_idx]),
            L1 * np.sin(theta1[frame_idx])
        ])
        
        joint2 = joint1 + np.array([
            L2 * np.cos(theta1[frame_idx] + theta2[frame_idx]),
            L2 * np.sin(theta1[frame_idx] + theta2[frame_idx])
        ])

        # make the cart a short line
        cart_line.set_data([cx - 0.1, cx + 0.1], [0.0, 0.0])
        
        # Draw pendulum 1 from the cart to the first point
        link1_line.set_data([base_pt[0], joint1[0]],
                            [base_pt[1], joint1[1]])
        
        # Draw pendulum 2 from the end of the first pendulum to the second one
        link2_line.set_data([joint1[0], joint2[0]],
                            [joint1[1], joint2[1]])

        # return each frame drawn
        return cart_line, link1_line, link2_line

    # basically put the animation together.
    ani = FuncAnimation(
        fig,
        update,
        frames=len(x),
        interval=interval,
        blit=True
    )
    # plot it
    plt.show()

    # return it
    return ani