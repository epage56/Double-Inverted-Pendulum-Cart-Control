import matplotlib.animation as animation
import numpy as np
from gekko import GEKKO

# Defining the gekko model
m = GEKKO(remote=True)

# Initial Conditions
x0 = 0; xdot0 = 0 # Cart position, velocity
theta10 = np.pi; theta1dot0 = 0 # Cart position, velocity; 0=vertical, pi=inverted
theta20 = np.pi; theta2dot0 = 0 # Cart position, velocity; 0=vertical, pi=inverted

# Final conditions
xf = 0; xdotf = 0 # Cart position, velocity initial conditions
theta1f = 0; theta1dotf = 0 
theta2f = 0; theta2dotf = 0 

# Other bcs
xmin = -0.070; xmax = 0.070 # Maximum cart travel limits (140mm of 150mm track used)
umin = -1; umax = 1 # Maximum force output (in N)

# Defining the time parameter (0, 1)
N = 100 # number of time steps
t = np.linspace(0,1,N)
m.time = t # initializing time in the model
 
# Final time
TF = m.FV(12,lb=2,ub=25); TF.STATUS = 1 # This sets the lower and upper acceptable solution times
final = np.zeros(len(m.time)) # pre-allocating the time array

for i in range(N):
    if i >=(N-2):
        final[i] = 1000

final = m.Param(value=final)

# Defining parameters about the model 

mc = m.Param(value=0.100) # cart mass, kg
m1 = m.Param(value=0.025) # link 1 mass, kg
m2 = m.Param(value=0.025) # link 2 mass, kg
L1 = m.Param(value=0.050) # link 1 length, m
LC1 = m.Param(value=0.025)  # link 1 CM pos, m
L2 = m.Param(value=0.050) # link 2 length, m
LC2 = m.Param(value=0.025) # link 2 CM pos, m
I1 = m.Param(value=.01) # link 1 MOI
I2 = m.Param(value=.01) # link 2 MOI
g = m.Const(value=9.81) # gravity, (m/s^2)
Bc = m.Const(value=.5) # cart estimated friction
B1 = m.Const(value=.001) # link 1 estimated friction 
B2 = m.Const(value=.001) # link 2 estimated friction

# Defining the manipulated variable (the force on the cart), maximum and minimum forces defined as well
u = m.MV(lb=umin,ub=umax); u.STATUS = 1 # minimum and maximum forces defined.

# defining the state variables
x, xdot, theta1, theta1dot, theta2, theta2dot = m.Array(m.Var, 6)

# Actually defining the initial conditions and bounds

# Cart
x.value = x0
xdot.value = xdot0

# Theta1
theta1.value = theta10
theta1dot.value = theta1dot0

# Theta2
theta2.value = theta10 
theta2dot.value = theta2dot0
x.LOWER = xmin; x.UPPER = xmax

# Derive state space for system
# Intermediates - this is to help simplify eqns when differentiation is performed 
h1 = m.Intermediate(mc + m1 + m2)
h2 = m.Intermediate(m1*LC1 + m2*L1)
h3 = m.Intermediate(m2*LC2)
h4 = m.Intermediate(m1*LC1**2 + m2*L1**2 + I1)
h5 = m.Intermediate(m2*LC2*L1)
h6 = m.Intermediate(m2*LC2**2 + I2)
h7 = m.Intermediate(m1*LC1*g + m2*L1*g)
h8 = m.Intermediate(m2*LC2*g)

# Put eqns in matrix form
M = np.array([[h1, h2*m.cos(theta1), h3*m.cos(theta2)],
              [h2*m.cos(theta1), h4, h5*m.cos(theta1-theta2)],
              [h3*m.cos(theta2), h5*m.cos(theta1-theta2), h6]])

C = np.array([[Bc, -h2*theta1dot*m.sin(theta1), -h3*theta2dot*m.sin(theta2)],
              [0, B1+B2, h5*theta2dot*m.sin(theta1-theta2)-B2],
              [0, -h5*theta1dot*m.sin(theta1-theta2)-B2, B2]])

G = np.array([0, -h7*m.sin(theta1), -h8*m.sin(theta2)])
U = np.array([u, 0, 0])
DQ = np.array([xdot, theta1dot, theta2dot])
CDQ = C@DQ
b = np.array([xdot.dt()/TF, theta1dot.dt()/TF, theta2dot.dt()/TF])
Mb = M@b

#Defining the State Space Model
m.Equations([(Mb[i] == U[i] - CDQ[i] - G[i]) for i in range(3)])
m.Equation(x.dt()/TF == xdot)
m.Equation(theta1.dt()/TF == theta1dot)
m.Equation(theta2.dt()/TF == theta2dot)

# End of SS and EOM

# Define penelization functions for wrong final state conditions

m.Obj(final*(x-xf)**2)
m.Obj(final*(xdot-xdotf)**2)
m.Obj(final*(theta1-theta1f)**2)
m.Obj(final*(theta1dot-theta1dotf)**2)
m.Obj(final*(theta2-theta2f)**2)
m.Obj(final*(theta2dot-theta2dotf)**2)

# Try to minimize final time - this is wack - minimize the integral of force^2 instead
# m.Obj(TF)
m.Obj(.05*u**2)

m.options.IMODE = 6 #MPC
m.options.SOLVER = 3 #IPOPT
m.solve()

m.time = np.multiply(TF, m.time)

print('Final time: ', TF.value[0])

print(theta1dot.value)

# All of the stuff below is plotting the results
import matplotlib.pyplot as plt

plt.close('all')

fig1 = plt.figure()
fig2 = plt.figure()
ax1 = fig1.add_subplot()
ax2 = fig2.add_subplot(321)
ax3 = fig2.add_subplot(322)
ax4 = fig2.add_subplot(323)
ax5 = fig2.add_subplot(324)
ax6 = fig2.add_subplot(325)
ax7 = fig2.add_subplot(326)

ax1.plot(m.time,u.value,'m',lw=2)
ax1.legend([r'$u$'],loc=1)
ax1.set_title('Control Input')
ax1.set_ylabel('Force (N)')
ax1.set_xlabel('Time (s)')
ax1.set_xlim(m.time[0],m.time[-1])
ax1.grid(True)

ax2.plot(m.time,x.value,'r',lw=2)
ax2.set_ylabel('Position (m)')
ax2.set_xlabel('Time (s)')
ax2.legend([r'$x$'],loc='upper left')
ax2.set_xlim(m.time[0],m.time[-1])
ax2.grid(True)
ax2.set_title('Cart Position')

ax3.plot(m.time,xdot.value,'g',lw=2)
ax3.set_ylabel('Velocity (m/s)')
ax3.set_xlabel('Time (s)')
ax3.legend([r'$xdot$'],loc='upper left')
ax3.set_xlim(m.time[0],m.time[-1])
ax3.grid(True)
ax3.set_title('Cart Velocity')

theta1alt  = np.zeros((N,1)); theta2alt  = np.zeros((N,1));
for i in range(N):
    theta1alt[i] = theta1.value[i] + np.pi/2
    theta2alt[i] = theta2.value[i] + np.pi/2

ax4.plot(m.time,theta1alt,'r',lw=2)
ax4.set_ylabel('Angle (Rad)')
ax4.set_xlabel('Time (s)')
ax4.legend([r'$theta1$'],loc='upper left')
ax4.set_xlim(m.time[0],m.time[-1])
ax4.grid(True)
ax4.set_title('Link 1 Position')

ax5.plot(m.time,theta1dot.value,'g',lw=2)
ax5.set_ylabel('Angular Velocity (Rad/s)')
ax5.set_xlabel('Time (s)')
ax5.legend([r'$theta1dot$'],loc='upper right')
ax5.set_xlim(m.time[0],m.time[-1])
ax5.grid(True)
ax5.set_title('Link 1 Velocity')

ax6.plot(m.time,theta2alt,'r',lw=2)
ax6.set_ylabel('Angle (Rad)')
ax6.set_xlabel('Time (s)')
ax6.legend([r'$theta2$'],loc='upper left')
ax6.set_xlim(m.time[0],m.time[-1])
ax6.grid(True)
ax6.set_title('Link 2 Position')

ax7.plot(m.time,theta2dot.value,'g',lw=2)
ax7.set_ylabel('Angular Velocity (Rad/s)')
ax7.set_xlabel('Time (s)')
ax7.legend([r'$theta2dot$'],loc='upper right')
ax7.set_xlim(m.time[0],m.time[-1])
ax7.grid(True)
ax7.set_title('Link 2 Velocity')

plt.rcParams['animation.html'] = 'html5'

x1 = x.value
y1 = np.zeros(len(m.time))

x2 = L1.value*np.sin(theta1.value)+x1
x2b = (1.05*L1.value[0])*np.sin(theta1.value)+x1
y2 = L1.value[0]*np.cos(theta1.value)+y1
y2b = (1.05*L1.value[0])*np.cos(theta1.value)+y1

x3 = L2.value[0]*np.sin(theta2.value)+x2
x3b = (1.05*L2.value[0])*np.sin(theta2.value)+x2
y3 = L2.value[0]*np.cos(theta2.value)+y2
y3b = (1.05*L2.value[0])*np.cos(theta2.value)+y2

fig = plt.figure(figsize=(8,6.4))
ax = fig.add_subplot(111,autoscale_on=False,\
                      xlim=(xmin,xmax),ylim=(L1*4,-L1*4))
ax.set_xlabel('position')
ax.set_aspect('equal')
ax.get_yaxis().set_visible(False)

crane_rail, = ax.plot([-2.5,2.5],[-0.2,-0.2],'k-',lw=4)
start, = ax.plot([-1.5,-1.5],[-1.5,1.5],'k:',lw=2)
objective, = ax.plot([1.5,1.5],[-1.5,1.5],'k:',lw=2)
mass1, = ax.plot([],[],linestyle='None',marker='s',\
                  markersize=40,markeredgecolor='k',\
                  color='orange',markeredgewidth=2)
mass2, = ax.plot([],[],linestyle='None',marker='o',\
                  markersize=20,markeredgecolor='k',\
                  color='orange',markeredgewidth=2)
mass3, = ax.plot([],[],linestyle='None',marker='o',\
                  markersize=20,markeredgecolor='k',\
                  color='orange',markeredgewidth=2)
line12, = ax.plot([],[],'o-',color='black',lw=4,\
                 markersize=6,markeredgecolor='k',\
                 markerfacecolor='k')
line23, = ax.plot([],[],'o-',color='black',lw=4,\
                 markersize=6,markeredgecolor='k',\
                 markerfacecolor='k')

time_template = 'time = %.1fs'
time_text = ax.text(0.05,0.9,'',transform=ax.transAxes)
#start_text = ax.text(-1.1,-0.3,'start',ha='right')
#end_text = ax.text(1.0,-0.3,'end',ha='left')

def init():
     mass1.set_data([],[])
     mass2.set_data([],[])
     mass3.set_data([],[])
     line12.set_data([],[])
     line23.set_data([],[])
     time_text.set_text('')
     return line12, line23, mass1, mass2, mass3, time_text

def animate(i):
     mass1.set_data([x1[i]], [y1[i]-0.1])
     mass2.set_data([x2[i]], [y2[i]])
     mass3.set_data([x3[i]], [y3[i]])
     line12.set_data([x1[i],x2[i]],[y1[i],y2[i]])
     line23.set_data([x2[i],x3[i]],[y2[i],y3[i]])
     time_text.set_text(time_template % m.time[i])
     return line12, line23, mass1, mass2, mass3, time_text

ani_a = animation.FuncAnimation(fig, animate, \
         np.arange(len(m.time)), \
         interval=40,init_func=init) #blit=False,

plt.show()

#Export Data
import csv

with open('U.csv', 'w', newline = '') as csvfile:
    my_writer = csv.writer(csvfile, delimiter = ' ')

    for i in range(N):
        input = np.array((m.time[i], u.value[i], x.value[i], theta1.value[i], theta2.value[i]))
        my_writer.writerow(input)

    csvfile.close