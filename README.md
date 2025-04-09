# Double Inverted Pendulum: Swing-Up and Control 

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Gekko](https://img.shields.io/badge/Gekko-Latest-green.svg)](https://gekko.readthedocs.io/)
[![Control](https://img.shields.io/badge/Control-Latest-orange.svg)](https://python-control.readthedocs.io/)

<p align="center">
  <img src="Media/DIPCdiagram.png" alt="Double Inverted Pendulum Diagram" width = "600">
</p>

## Table of Contents
- [Overview](#-overview)
- [Background](#-background)
- [Project Components](#-project-components)
- [Technical Approach](#-technical-approach)
- [Installation](#-installation)
- [Usage](#-usage)
- [Results](#-results)
- [Dependencies](#-dependencies)
- [Contributing](#-contributing)
- [License](#-license)
- [Acknowledgments](#-acknowledgments)
- [Contact](#-contact)

## Overview

This repository contains an implementation of trajectory optimization and optimal control theory in python for the double inverted pendulum cart. The project uses nonlinear optimization to find the optimal force input on the system to achieve the swing up of the pendulum and then uses an LQR controller to stay in the inverted position. The project includes an animation and graphs to visualize the dynamics of the pendulum as time evolves. 

## Background

### What is a Double Inverted Pendulum?

The double inverted pendulum cart (DIPC) is a classic example of a chaotic system in physics and is used in academic control theory and robotics to benchmark different control strategies. Control theorists are interested in the system because it is an underactuated system, meaning that it has more degrees of freedom (3, theta1, theta2, and x) than its number of independently controllable actuators (1, u). 

There are two common benchmarks of control with this system. The first is keeping the DIPC in the inverted position despite small disturbances while the second type of benchmark involves moving the system from one state to another state. Commonly the states chosen are two different cart positions with the pendulum inverted or swinging the system up from the stable bottom equilibrium to the unstable inverted position. 

<details>
<summary>Derivation of Dynamics Equations (Click to expand)</summary>

The equation of motion describing the double inverted pendulum can be derived with lagrangian or newtonian mechanics. Lagrangian mechanics were chosen for this problem because it is more interpretable. For more background on this subject I recommend the text - Modern Robotics: Mechanics, Planning, and Control by Park and Lynch. 

The lagrangian is given as:

$$L = T - V$$

Where:
- $T$ is the kinetic energy of the system
- $V$ is the potential energy of the system

For the double pendulum on a cart, the Lagrangian leads to a set of coupled differential equations:

$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = \tau$$

Where:
- $q = [x, \theta_1, \theta_2]^T$ is the vector of generalized coordinates
- $M(q)$ is the mass matrix
- $C(q,\dot{q})$ accounts for Coriolis and centrifugal effects
- $G(q)$ represents gravitational forces
- $\tau = [F, 0, 0]^T$ is the generalized force vector with $F$ being the control input

</details>

## 🧩 Project Components

```mermaid
graph TD
    A[System Modeling] --> B[Swing-Up Optimization]
    B --> C[LQR Controller]
    A --> C
    C --> D[Simulation Environment]
    B --> D
```

1. **System Modeling**: Derivation of the dynamic equations of motion
2. **Swing-Up Optimization**: Using Gekko to solve the nonlinear programming problem of optimal swing up
3. **LQR Controller**: Implementation of a Linear Quadratic Regulator for inverted balencing 
4. **Simulation Environment**: Visualization and analysis tools

## Technical Approach

### Dynamics Formulation

The mathematical model of the double pendulum cart system is derived using Lagrangian mechanics, resulting in coupled differential equations that describe the motion of the system.

### Swing-Up Strategy

The swing-up problem is formulated as an optimal control problem and solved using Gekko, a Python package for nonlinear optimization. The objective is to find a control input sequence that moves the pendulums from the downward position to the upright position while minimizing a cost function (typically energy or time).

<details>
<summary> Nonlinear Optimization Example</summary>

```python
# Example of setting up the optimization problem in Gekko
from gekko import GEKKO

# Initialize Gekko model
m = GEKKO()

# Time steps
n = 100
m.time = np.linspace(0, 5, n)

# Variables
x = m.Var(value=0)     # Cart position
theta1 = m.Var(value=np.pi)  # First pendulum angle (starting downward)
theta2 = m.Var(value=np.pi)  # Second pendulum angle (starting downward)

# Control input
u = m.MV(value=0, lb=-10, ub=10)  # Force on cart
u.STATUS = 1  # Allow optimizer to change this value

# Dynamics (simplified example)
m.Equation(x.dt() == ...)
m.Equation(theta1.dt() == ...)
m.Equation(theta2.dt() == ...)

# Objective function: Minimize time to upright position + control effort
m.Obj(sum((theta1-0)**2 + (theta2-0)**2 + 0.1*u**2))

# Solve
m.options.IMODE = 6  # Dynamic optimization
m.solve()
```
</details>

### LQR Control

Once near the upright position, a Linear Quadratic Regulator (LQR) takes over to stabilize the system. The LQR design involves:
- Linearizing the system around the upright equilibrium point
- Selecting appropriate state and control weight matrices (Q and R)
- Solving the Riccati equation to obtain the optimal feedback gain matrix

<details>
<summary>LQR Implementation Example</summary>

```python
# Example of implementing LQR control
import numpy as np
import control as ctrl

# Linearized system at the upright equilibrium point
# State vector: [x, x_dot, theta1, theta1_dot, theta2, theta2_dot]
A = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 0, a1, 0, a2, 0],
    [0, 0, 0, 1, 0, 0],
    [0, 0, a3, 0, a4, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, a5, 0, a6, 0]
])

# Input matrix
B = np.array([[0], [b1], [0], [b2], [0], [b3]])

# LQR weight matrices
Q = np.diag([1, 1, 10, 1, 10, 1])  # State cost
R = np.array([[0.1]])              # Control cost

# Solve the Riccati equation to get the optimal gain matrix K
K, S, E = ctrl.lqr(A, B, Q, R)

# Control law: u = -K*x
def lqr_control(state):
    return -np.dot(K, state)
```
</details>

## Installation

```bash
# Clone the repository
git clone https://github.com/yourusername/double-pendulum-control.git
cd double-pendulum-control

# Create and activate a virtual environment (optional but recommended)
python -m venv env
source env/bin/activate  # On Windows: env\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## 📦 Dependencies

- Python 3.8+
- Gekko
- NumPy
- SciPy
- Matplotlib
- Control

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

