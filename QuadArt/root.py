# Variables and constants shared across all modules
import numpy as np

# model constants
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2 # length of arm

# Initial values
PosGi = np.array([0,0,0]) # global position
VelBi = np.array([0,0,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0]) # body angular velocity
PosInit = 0.5 * np.array([[-l,0,0,l,0],[0,l,-l,0,0],[0,0,0,0,0],[2,2,2,2,2]])

# marker geometry, cm
l1 = 6.0
l2 = 3.0

verts = np.array([
    [-l1/2, 0, -l1/2],
    [l1/2, 0, -l1/2],
    [-l1/2, 0, l1/2],
    [l1/2, 0, l1/2],
   [-l2/2, 0, -l2/2],
    [l2/2, 0, -l2/2],
    [-l2/2, 0, l2/2],
    [l2/2, 0, l2/2]
])

faces = np.array([
    [0, 1, 2],
    [3, 1, 2],
    [4, 5, 6],
    [7, 5, 6]
])
colors = np.array([
    [1, 1, 1, 0.2],
    [1, 1, 1, 0.2],
    [1, 1, 1, 0.5],
    [1, 1, 1, 0.5]
])

# Axes in 3D
AxesPos = np.array([[0, 0, 0], [1, 0, 0], [0, 0, 0],\
    [0, 1, 0], [0, 0, 0], [0, 0, 1]])
AxesColor = np.array([[0, 0, 1, 0.9], [0, 0, 1, 0.9],\
    [0, 1, 0, 0.9], [0, 1, 0, 0.9], [1, 0, 0, 0.9], [1, 0, 0, 0.9]])

# Crazyflie config
link_uri = 'radio://0/80/250K'

F = 0.0
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 0.0
k1 = 0.981; # coefficient which relates force with command signals
k2 = 1; # coefficient which relates torque with command signals
Tend = 5.0
dt = 0.02
t = 0
