import numpy as np
import matplotlib.pyplot as plt
from gekko import GEKKO

# Initialize model
m = GEKKO(remote=True)

# Time discretization
N = 100
t = np.linspace(0, 1, N)
m.time = t

# Final time to be optimized
TF = m.FV(value=10, lb=2, ub=25); TF.STATUS = 1
final = m.Param(value=[0 if i < N-2 else 1000 for i in range(N)])

# Parameters
mc = m.Param(value=1)  # cart mass
m1 = m.Param(value=1)  # pendulum mass
L1 = m.Param(value=1)  # pendulum length
LC1 = m.Param(value=0.5) # center of mass
I1 = m.Param(value=0.15)  # moment of inertia
g = m.Const(value=-9.81)
Bc = m.Const(value=0.2)    # cart friction
B1 = m.Const(value=0.001)  # pendulum friction

# Control input
u = m.MV(lb=-25, ub=25); u.STATUS = 1

# States
x, xdot, theta1, theta1dot = m.Array(m.Var, 4)
x.value = 0; x.LOWER = -2; x.UPPER = 2
xdot.value = 0
theta1.value = np.pi
theta1dot.value = 0

# Intermediates for dynamics
h1 = m.Intermediate(mc + m1)
h2 = m.Intermediate(m1 * LC1)
h4 = m.Intermediate(m1 * LC1**2 + I1)
h7 = m.Intermediate(m1 * LC1 * g)

M = np.array([[h1, -h2*m.cos(theta1)],
              [h2*m.cos(theta1), h4]])

C = np.array([[Bc, -h2*theta1dot*m.sin(theta1)],
              [0, B1]])

G = np.array([0, h7*m.sin(theta1)])
U = np.array([u, 0])
DQ = np.array([xdot, theta1dot])
b = np.array([xdot.dt()/TF, theta1dot.dt()/TF])
CDQ = C @ DQ
Mb = M @ b

# Equations of motion
m.Equations([(Mb[i] == U[i] - CDQ[i] - G[i]) for i in range(2)])
m.Equation(x.dt()/TF == xdot)
m.Equation(theta1.dt()/TF == theta1dot)

# Objective function
m.Obj(final*(x)**2)
m.Obj(final*(xdot)**2)
m.Obj(final*(theta1)**2)
m.Obj(final*(theta1dot)**2)

m.Obj(TF)
m.Obj(0.1*u**2)

# Solve
m.options.IMODE = 6
m.options.SOLVER = 3
m.solve()

# Rescale time for plotting
t_actual = np.multiply(TF.value[0], m.time)

# Plotting
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(t_actual, u.value, label='Control Input (N)', color='purple')
plt.ylabel('Force')
plt.legend(); plt.grid()

plt.subplot(3, 1, 2)
plt.plot(t_actual, x.value, label='Cart Position (m)', color='blue')
plt.ylabel('x (m)')
plt.legend(); plt.grid()

plt.subplot(3, 1, 3)
plt.plot(t_actual, theta1.value, label='Pendulum Angle (rad)', color='red')
plt.ylabel('θ (rad)')
plt.xlabel('Time (s)')
plt.legend(); plt.grid()

plt.tight_layout()
plt.show()


import matplotlib.animation as animation

# Extract solved states
x_vals = x.value
theta_vals = theta1.value
cart_y = np.zeros(len(x_vals))  # cart stays on rail

# Pendulum tip (end of link) position
x_pend = [x + L1.value[0]*np.sin(theta) for x, theta in zip(x_vals, theta_vals)]
y_pend = [L1.value[0]*np.cos(theta) for theta in theta_vals]

# Create figure
fig, ax = plt.subplots(figsize=(8, 5))
ax.set_xlim(-3, 3)
ax.set_ylim(-1.5, 1.5)
ax.set_aspect('equal')
ax.set_title('Single Pendulum Swing-Up')
ax.set_aspect('equal')
ax.grid()

# Plot elements
rail, = ax.plot([-0.15, 0.15], [0, 0], 'k-', lw=2)
cart, = ax.plot([], [], 's', markersize=20, color='orange', markeredgewidth=2, markeredgecolor='k')
pendulum, = ax.plot([], [], 'o-', color='black', lw=3, markersize=6, markerfacecolor='k', markeredgecolor='k')
time_text = ax.text(0.02, 0.9, '', transform=ax.transAxes)

def init():
    cart.set_data([], [])
    pendulum.set_data([], [])
    time_text.set_text('')
    return cart, pendulum, time_text

def animate(i):
    # Cart
    cart.set_data([x_vals[i]], [-0.01])
    # Pendulum line from cart to pendulum tip
    pendulum.set_data([x_vals[i], x_pend[i]], [0, y_pend[i]])
    time_text.set_text(f'time = {t_actual[i]:.2f} s')
    return cart, pendulum, time_text

ani = animation.FuncAnimation(fig, animate, frames=len(t_actual),
                              interval=40, blit=True, init_func=init)

plt.show()