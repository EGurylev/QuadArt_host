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

# marker constants
K = 88 # coefficient for Pinhole camera model
L = 0.08 # size of a marker, m

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
