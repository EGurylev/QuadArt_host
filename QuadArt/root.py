# Variables and constants shared across all modules
import numpy as np
import math

#Image resolution
W, H = 1280, 720

# marker geometry, cm
l1 = 6.0
l2 = 3.0

#Camera parameters
f = 1180.0
objectPoints = np.array([[-l2/2, -l2/2, 0],\
    [l2/2, -l2/2, 0],\
    [-l2/2, l2/2, 0], \
    [l2/2, l2/2, 0]])
cameraMatrix = np.array([[f, 0, W / 2],\
    [0, f, H / 2], [0, 0, 1]])
distCoeffs = np.zeros((5,1))

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

# Angles from IMU
roll_cf = 0
pitch_cf = 0
yaw_cf = 0
# Angles from estimation
roll_e = 0
pitch_e = 0
yaw_e = 0

# Marker points from camera
CornerCoord = np.zeros([4,2])

# Model parameters
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2 # length of arm
F = 0.0 # Force
# torques
tau_phi = 0.0
tau_theta = 0.0
tau_psi = 0.0

k1 = 0.981; # coefficient which relates force with command signals
k2 = 1; # coefficient which relates torque with command signals
dt = 0.02
t = 0

# Initial values
PosGi = np.array([0,0,0]) # global position
VelBi = np.array([0,0,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0]) # body angular velocity
PosInit = 0.5 * np.array([[-l,0,0,l,0],[0,l,-l,0,0],[0,0,0,0,0],[2,2,2,2,2]])

# Pose estimation parameters
tvec = np.zeros([3,1])
tvec[2] = 5.0
tvec_Prev = np.zeros([3,1])
tvec_Prev[2] = 5.0
rvec = np.zeros([3,1])

# Global function
def Euler2Mat(roll, pitch, yaw, order):
   Rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], \
       [0, math.sin(roll), math.cos(roll)]])
   Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0],  \
       [-math.sin(pitch), 0, math.cos(pitch)]])
   Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],  \
       [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
   if order == 'XYZ':
       R = np.dot(Rx,Ry)
       R = np.dot(R,Rz)
   elif order == 'ZYX':
       R = np.dot(Rz,Ry)
       R = np.dot(R,Rx)
   return R
