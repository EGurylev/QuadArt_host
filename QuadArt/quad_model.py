import root as r
from scipy import integrate
import numpy as np
import math


def rhs(y,t): 
    # return derivatives of the array y
    deriv = np.zeros(12)

    v_x, v_y, v_z = y[0], y[1], y[2] # linear velocities
    #X, Y, Z = y[3], y[4], y[5] # positions
    phi, theta, psi = y[6], y[7], y[8] # Euler angles
    omega_x, omega_y, omega_z = y[9], y[10], y[11] #angular velocity
    # Body linear velocities
    deriv[0] = omega_z * v_y - omega_y * v_z + r.g * math.sin(theta)
    deriv[1] = omega_x * v_z - omega_z * v_x - r.g * math.cos(theta) * math.sin(phi)
    deriv[2] = omega_y * v_x - omega_x * v_y - r.g * math.cos(theta) * math.cos(phi) + r.force / r.mass
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
    # Body angular velocities
    deriv[9] = (r.j_yy - r.j_zz) / r.j_xx * omega_y * omega_z + r.tau_phi / r.j_xx
    deriv[10] = (r.j_zz - r.j_xx) / r.j_yy * omega_x * omega_z + r.tau_theta / r.j_yy
    deriv[11] = (r.j_xx - r.j_yy) / r.j_zz * omega_x * omega_y + r.tau_psi / r.j_zz
    return deriv   
        

def homog_transform(y, pos_init, order):
   rot_mat = r.euler2mat(y[6], y[7], y[8], order)
   p = y[3:6]
   homog_mat = np.concatenate((rot_mat.T, [p]),axis=0)
   homog_mat = np.concatenate((homog_mat.T, [np.array([0, 0, 0, 1])]),axis=0)
   pos = np.dot(homog_mat, pos_init)
   return pos
