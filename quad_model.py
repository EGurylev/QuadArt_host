import root as r
from scipy import integrate
import numpy as np
import math

# model constants
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2 # length of arm
Jxx = M * math.pow(l,2); # moment of inertia according to x axis
Jyy = M * math.pow(l,2); # moment of inertia according to y axis
Jzz = 2 * M * math.pow(l,2); # moment of inertia according to z axis

def RHS(y,t): 
    # return derivatives of the array y
    deriv = np.zeros(12)

    v_x, v_y, v_z = y[0], y[1], y[2] # linear velocities
    X, Y, Z = y[3], y[4], y[5] # positions
    phi, theta, psi = y[6], y[7], y[8] # Euler angles
    omega_x, omega_y, omega_z = y[9], y[10], y[11] #angular velocity
    # Body linear velocities
    deriv[0] = omega_z * v_y - omega_y * v_z + g * math.sin(theta)
    deriv[1] = omega_x * v_z - omega_z * v_x - g * math.cos(theta) * math.sin(phi)
    deriv[2] = omega_y * v_x - omega_x * v_y - g * math.cos(theta) * math.cos(phi) + r.F / M
    # Global positions
    deriv[3] = math.cos(theta) * math.cos(psi) * v_x + \
         ( math.sin(phi) * math.sin(theta) * math.cos(psi) - math.cos(phi) * math.sin(psi) ) * v_y + \
                       ( math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi) ) * v_z
    deriv[4] = math.cos(theta) * math.sin(psi) * v_x + \
          ( math.sin(phi) * math.sin(theta) * math.sin(psi) + math.cos(phi) * math.cos(psi) ) * v_y + \
          ( math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi) * math.cos(psi) ) * v_z
    deriv[5] = -math.sin(theta) * v_x + math.sin(phi) * math.cos(theta) * v_y + \
          math.cos(phi) * math.cos(theta) * v_z
    # Global angles
    deriv[6] = omega_x + math.sin(phi) * math.tan(theta) * omega_y + math.cos(phi) * math.tan(theta) * omega_z
    deriv[7] = math.cos(phi) * omega_y - math.sin(phi) * omega_z
    deriv[8] = math.sin(phi) / math.cos(theta) * omega_y + math.cos(phi) / math.cos(theta) * omega_z
    # Body angiular velocities
    deriv[9] = (Jyy - Jzz) / Jxx * omega_y * omega_z + r.tau_phi / Jxx
    deriv[10] = (Jzz - Jxx) / Jyy * omega_x * omega_z + r.tau_theta / Jyy
    deriv[11] = (Jxx - Jyy) / Jzz * omega_x * omega_y + r.tau_psi / Jzz
    return deriv   
        

def AffineTransform(y, PosInit):
   Rx = np.array([[1, 0, 0], [0, math.cos(y[6]), -math.sin(y[6])], \
       [0, math.sin(y[6]), math.cos(y[6])]])
   Ry = np.array([[math.cos(y[7]), 0, math.sin(y[7])], [0, 1, 0],  \
       [-math.sin(y[7]), 0, math.cos(y[7])]])
   Rz = np.array([[math.cos(y[8]), -math.sin(y[8]), 0],  \
       [math.sin(y[8]), math.cos(y[8]), 0], [0, 0, 1]])
   R = np.dot(Rx,Ry)
   R = np.dot(R,Rz)
   P = y[3:6]
   T1 = np.concatenate((R.T, [P]),axis=0)
   T1 = np.concatenate((T1.T, [np.array([0, 0, 0, 1])]),axis=0)
   pos = np.dot(T1, PosInit)
   return pos
