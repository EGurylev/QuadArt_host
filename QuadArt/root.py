# Variables and constants shared across all modules
import numpy as np
import math
from geometry import * 

#Image resolution
w, h = 1280, 720

#Camera parameters
f = 1180.0
object_points = np.array([[-l2/2, -l2/2, 0],\
    [l2/2, -l2/2, 0],\
    [-l2/2, l2/2, 0], \
    [l2/2, l2/2, 0]])
camera_matrix = np.array([[f, 0, w / 2],\
    [0, f, h / 2], [0, 0, 1]])
dist_coeffs = np.zeros((5,1))
    

# Crazyflie config
link_uri = 'radio://0/80/250K'
thrust_eq = 40000 # equilibrium point

# Angles from IMU
roll_cf = 0
pitch_cf = 0
yaw_cf = 0
# Angles from estimation
roll_e = 0
pitch_e = 0
yaw_e = 0
# Angles set
roll_set = 0
pitch_set = 0
yaw_set = 0
# Pose controller parameters
thrust_set = 0
thrust_cf = 0

# Marker points from camera
corner_coord = np.zeros([4,2])

# Model parameters
g = 9.81 # gravity acceleration
mass = 0.4 # quadrotor's mass
l = 0.2 # length of arm
force = 0.0 # Force
j_xx = mass * math.pow(l,2); # moment of inertia according to x axis
j_yy = mass * math.pow(l,2); # moment of inertia according to y axis
j_zz = 2 * mass * math.pow(l,2); # moment of inertia according to z axis
# torques
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 0.0

k1 = 0.981; # coefficient which relates force with command signals
k2 = 1; # coefficient which relates torque with command signals
dt = 0.02
t = 0

# Initial values
pos_gi = np.array([0,0,0]) # global position
vel_bi = np.array([0,0,0]) # body linear velocity
angles_gi = np.array([0,0,0]) # global angles (Euler)
omega_bi = np.array([0,0,0]) # body angular velocity
pos_init = 0.5 * np.array([[-l,0,0,l,0],[0,l,-l,0,0],[0,0,0,0,0],[2,2,2,2,2]])

# Pose estimation parameters
tvec = np.zeros([3,1])
tvec[2] = 5.0
tvec_prev = np.zeros([3,1])
tvec_prev[2] = 5.0
rvec = np.zeros([3,1])
marker_found = False
ramp = 1000

# Global function
def euler2mat(roll, pitch, yaw, order):
   rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], \
       [0, math.sin(roll), math.cos(roll)]])
   ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],  \
       [-math.sin(pitch), 0, math.cos(pitch)]])
   rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],  \
       [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
   if order == 'XYZ':
       r = np.dot(rx,ry)
       r = np.dot(r,rz)
   elif order == 'ZYX':
       r = np.dot(rz,ry)
       r = np.dot(r,rx)
   return r
