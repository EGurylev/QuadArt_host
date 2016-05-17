import root as r
import control.matlab as ml
import matplotlib.pyplot as plt
import sympy as sp
import numpy as np
from my_functions import *


# Lookup tables for thrust
pwm_table = 650 * np.array([0,6.25,12.5,18.75,25,31.25,37.5,43.25,50,56.25,
    62.5,68.75,75,81.25,87.5,93.75]) # in cf's thrust control range
rpm_table = np.array([0,4485,7570,9374,10885,12277,13522,14691,15924,17174,
    18179,19397,20539,21692,22598,23882]) # revolutions per minute
thrust_table = np.array([0,1.6,4.8,7.9,10.9,13.9,17.3,21.0,24.4,28.6,32.8,
    37.3,41.7,46.0,51.9,57.9]) / 1e3 # kg
# Calc. rpm and pwm output for equilibrium point   
rpm_eq = np.interp(r.mass, thrust_table, rpm_table)
pwm_eq = np.interp(rpm_eq, rpm_table, pwm_table)

# Quadrotor model in sympy
x, y, z = sp.symbols('x, y, z', real=True)
v_x, v_y, v_z = sp.symbols('v_x, v_y, v_z', real=True)
phi, theta, psi = sp.symbols('phi, theta, psi', real=True)
omega_x, omega_y, omega_z = sp.symbols('omega_x, omega_y, omega_z', real=True)

# Body linear velocities    
f1 = omega_z * v_y - omega_y * v_z + r.g * sp.sin(theta)
f2 = omega_x * v_z - omega_z * v_x - r.g * sp.cos(theta) * sp.sin(phi)
f3 = omega_y * v_x - omega_x * v_y - r.g * sp.cos(theta) * sp.cos(phi) + \
    r.force / r.mass
# Global positions
f4 = sp.cos(theta) * sp.cos(psi) * v_x + \
    (sp.sin(phi) * sp.sin(theta) * sp.cos(psi) - sp.cos(phi) * sp.sin(psi)) * v_y + \
    (sp.cos(phi) * sp.sin(theta) * sp.cos(psi) + sp.sin(phi) * sp.sin(psi)) * v_z
f5 = sp.cos(theta) * sp.sin(psi) * v_x + \
    (sp.sin(phi) * sp.sin(theta) * sp.sin(psi) + sp.cos(phi) * sp.cos(psi)) * v_y + \
    (sp.cos(phi) * sp.sin(theta) * sp.sin(psi) - sp.sin(phi) * sp.cos(psi)) * v_z
f6 = -sp.sin(theta) * v_x + sp.sin(phi) * sp.cos(theta) * v_y + \
    sp.cos(phi) * sp.cos(theta) * v_z  
# Global angles
f7 = omega_x + sp.sin(phi) * sp.tan(theta) * omega_y + sp.cos(phi) * \
    sp.tan(theta) * omega_z
f8 = sp.cos(phi) * omega_y - sp.sin(phi) * omega_z
f9 = sp.sin(phi) / sp.cos(theta) * omega_y + sp.cos(phi) \
    / sp.cos(theta) * omega_z
# Body angular velocities
f10 = (r.j_yy - r.j_zz) / r.j_xx * omega_y * omega_z + r.tau_phi / r.j_xx
f11 = (r.j_zz - r.j_xx) / r.j_yy * omega_x * omega_z + r.tau_theta / r.j_yy
f12 = (r.j_xx - r.j_yy) / r.j_zz * omega_x * omega_y + r.tau_psi / r.j_zz

# Linearize quadrotor model
F = sp.Matrix([f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12])
J = F.jacobian([v_x, v_y, v_z, x, y, z, phi, theta, psi, omega_x, omega_y, omega_z])
# Substitute linearization point and obtain matrix A (state-space form)
A = J.subs([(v_x, 0), (v_y, 0), (v_z, 0), (phi, 0), (theta, 0), (psi, 0),
    (omega_x, 0), (omega_y, 0), (omega_z, 0)])
A = np.array(A)

# Model of feedback loops of Crazyflie
attitude_rate = 500 # Hz
dt = 1.0 / attitude_rate
# Attitude control consist of 2 feedback loops in cascade: first loop controls angle
# and outputs desired rate of angle. Coefficients taken from the last CF's firmware
pid_yaw_rate_kp, pid_yaw_rate_ki, pid_yaw_rate_kd = 70, 16.7, 0
pid_roll_rate_kp, pid_roll_rate_ki, pid_roll_rate_kd = 70, 0, 0
pid_pitch_rate_kp, pid_pitch_rate_ki, pid_pitch_rate_kd = 70, 0, 0

pid_yaw_kp, pid_yaw_ki, pid_yaw_kd = 10, 1.0, 0.35
pid_roll_kp, pid_roll_ki, pid_roll_kd = 3.5, 2, 0
pid_pitch_kp, pid_pitch_ki, pid_pitch_kd = 3.5, 2, 0

# Yaw pid controller
yaw_pid_d = pid_yaw_kp + pid_yaw_ki * dt * ml.tf([1, 0], [1, -1], dt) + \
    (pid_yaw_kd / dt) * (1 - ml.tf(1, [1, 0], dt))
yaw_pid_c = d2c(yaw_pid_d, 'bi')# continious version
# Yaw rate pid controller
yaw_rate_pid_d = pid_yaw_rate_kp + pid_yaw_rate_ki * dt * ml.tf([1, 0], [1, -1], dt) + \
    (pid_yaw_rate_kd / dt) * (1 - ml.tf(1, [1, 0], dt))
yaw_rate_pid_c = d2c(yaw_rate_pid_d, 'bi')# continious version

# Roll pid controller
roll_pid_d = pid_roll_kp + pid_roll_ki * dt * ml.tf([1, 0], [1, -1], dt) + \
    (pid_roll_kd / dt) * (1 - ml.tf(1, [1, 0], dt))
roll_pid_c = d2c(roll_pid_d, 'bi')# continious version
# Roll rate pid controller
pid_roll_rate_kp# only proportional part

# Pitch pid controller
pitch_pid_d = pid_pitch_kp + pid_pitch_ki * dt * ml.tf([1, 0], [1, -1], dt) + \
    (pid_pitch_kd / dt) * (1 - ml.tf(1, [1, 0], dt))
pitch_pid_c = d2c(pitch_pid_d, 'bi')# continious version
# Pitch rate pid controller
pid_pitch_rate_kp# only proportional part

