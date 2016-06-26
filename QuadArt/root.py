'''
Module which contains all global parameters and used as exchange point.
'''

# Variables and constants shared across all modules
import numpy as np
import math
from geometry import * 

#Image resolution
w, h = 1280, 1024

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
thrust_eq = 40500 # equilibrium point

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
# Battery voltage
vbat = 0

# Marker points from camera
corner_coord = np.zeros([4,2])

# Model parameters
g = 9.81 # gravity acceleration
mass = 0.029 # quadrotor's mass + mass of a marker
l = 0.045 # length of arm
force = 0.0 # Force
j_xx = 2.3951e-5 # moment of inertia according to x axis
j_yy = 2.3951e-5 # moment of inertia according to y axis
j_zz = 3.2347e-5 # moment of inertia according to z axis
# torques
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 0.0

# Lookup tables for thrust
pwm_table = 650 * np.array([0,6.25,12.5,18.75,25,31.25,37.5,43.25,50,56.25,
    62.5,68.75,75,81.25,87.5,93.75]) # in cf's thrust control range
rpm_table = np.array([0,4485,7570,9374,10885,12277,13522,14691,15924,17174,
    18179,19397,20539,21692,22598,23882]) # revolutions per minute
thrust_table = np.array([0,1.6,4.8,7.9,10.9,13.9,17.3,21.0,24.4,28.6,32.8,
    37.3,41.7,46.0,51.9,57.9]) / 1e3 # kg
# Thrust vs voltage polynomial fit (polynomial coefficients):
thrust_volt_fit = np.array([3.747, 5.804, 0.745]) # volts to grams

k1 = 0.005022; # coefficient which relates force with command signals (consider to remove it)
k2 = 1.858e-10; # coefficient which relates torque with command signals (should be tuned)
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
