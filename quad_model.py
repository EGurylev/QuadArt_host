from scipy import integrate
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.art3d as art3d
import math
import time


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
N = 100
Tend = 1.0
Time = np.linspace(0.0,Tend,N)

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
VelBi = np.array([0,0.1,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0.3]) # body angular velocity
yinit = np.concatenate([VelBi, PosGi, AnglesGi, OmegaBi])

# forces and moments
F = 10
tau_phi = 0.1
tau_theta = 0.1
tau_psi = 0.1

y = integrate.odeint(RHS,yinit,Time)

# display simulation results

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

ax.set_xlim3d(min(y[:,3]), max(y[:,3]))
ax.set_ylim3d(min(y[:,4]), max(y[:,4]))
ax.set_zlim3d(min(y[:,5]), max(y[:,5]))

PosInit = 0.5 * np.array([[-l,0,0,l,0],[0,l,-l,0,0],[0,0,0,0,0],[2,2,2,2,2]])
q, = plt.plot(PosInit[0], PosInit[1], PosInit[2], 'o')

for n in xrange(N):
    Rx = np.array([[1, 0, 0], [0, math.cos(y[n,6]), -math.sin(y[n,6])], [0, math.sin(y[n,6]), math.cos(y[n,6])]])
    Ry = np.array([[math.cos(y[n,7]), 0, math.sin(y[n,7])], [0, 1, 0], [-math.sin(y[n,7]), 0, math.cos(y[n,7])]])
    Rz = np.array([[math.cos(y[n,8]), -math.sin(y[n,8]), 0], [math.sin(y[n,8]), math.cos(y[n,8]), 0], [0, 0, 1]])
    R = np.dot(Rx,Ry)
    R = np.dot(R,Rz)
    P = y[n,3:6]
    T1 = np.concatenate((R.T, [P]),axis=0)
    T1 = np.concatenate((T1.T, [np.array([0, 0, 0, 1])]),axis=0)
    
    Pos = np.dot(T1, PosInit)
    q.set_data(Pos[0:2])
    q.set_3d_properties(Pos[2])
    plt.draw()
    time.sleep(Tend/N)


