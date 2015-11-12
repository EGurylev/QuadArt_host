from scipy import integrate
import numpy as np
import matplotlib.pyplot as plt
import math


def RHS(y,t): 
    # return derivatives of the array y
    deriv = np.zeros(12)
    
    v_x, v_y, v_z = y[0], y[1], y[2] # linear velocities
    X, Y, Z = y[3], y[4], y[5] # positions
    phi, theta, psi = y[6], y[7], y[8] # Euler angles
    omega_x, omega_y, omega_z = y[9], y[10], y[11] #angular velocity
    
    # Linear velocities
    deriv[0] = omega_z * v_y - omega_y * v_z + g * math.sin(theta)
    deriv[1] = omega_x * v_z - omega_z * v_x - g * math.cos(theta) * math.sin(phi)
    deriv[2] = omega_y * v_x - omega_x * v_y - g * math.cos(theta) * math.cos(phi) + F / M

    deriv[3] = math.cos(theta) * math.cos(psi) * v_x + \
               ( math.sin(phi) * math.sin(theta) * math.cos(psi) - math.cos(phi) * math.sin(psi) ) * v_y + \
               ( math.cos(phi) * math.sin(theta) * math.cos(psi) + math.sin(phi) * math.sin(psi) ) * v_z
    deriv[4] = math.cos(theta) * math.sin(psi) * v_x + \
               ( math.sin(phi) * math.sin(theta) * math.sin(psi) + math.cos(phi) * math.cos(psi) ) * v_y + \
               ( math.cos(phi) * math.sin(theta) * math.sin(psi) - math.sin(phi) * math.cos(psi) ) * v_z
    deriv[5] = -math.sin(theta) * v_x + math.sin(phi) * math.cos(theta) * v_y + \
               math.cos(phi) * math.cos(theta) * v_z

    deriv[6] = omega_x + math.sin(phi) * math.tan(theta) * omega_y + math.cos(phi) * math.tan(theta) * omega_z
    deriv[7] = math.cos(phi) * omega_y - math.sin(phi) * omega_z
    deriv[8] = math.sin(phi) / math.cos(theta) * omega_y + math.cos(phi) / math.cos(theta) * omega_z

    deriv[9] = (Jyy - Jzz) / Jxx * omega_y * omega_z + tau_phi / Jxx
    deriv[10] = (Jzz - Jxx) / Jyy * omega_x * omega_z + tau_theta / Jyy
    deriv[11] = (Jxx - Jyy) / Jzz * omega_x * omega_y + tau_psi / Jzz



    return deriv

# simulation time
time = np.linspace(0.0,10.0,1000)

# model constants
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2; # length of arm
Jxx = M * math.pow(l,2); # moment of inertia according to x axis
Jyy = M * math.pow(l,2); # moment of inertia according to y axis
Jzz = 2 * M * math.pow(l,2); # moment of inertia according to z axis
k1 = 0.981; # coefficient which relates force with command signals
k2 = 1; # coefficient which relates torque with command signals

# initial values
PosGi = np.array([0,0,0]) # global position
VelBi = np.array([0.5,0,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0.4]) # body angular velocity
yinit = np.concatenate([VelBi, PosGi, AnglesGi, OmegaBi])

# forces and moments
F = 10
tau_phi = 0.1
tau_theta = 0.25
tau_psi = 0.1

y = integrate.odeint(RHS,yinit,time)

# display simulation results
plt.plot(time,y[:,0])
