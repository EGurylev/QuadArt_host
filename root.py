# Variables and constants shared across all modules

# model constants
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2 # length of arm

F = 0.0
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 0.0
k1 = 0.981; # coefficient which relates force with command signals
k2 = 1; # coefficient which relates torque with command signals
Tend = 5.0
dt = 0.035
t = 0
