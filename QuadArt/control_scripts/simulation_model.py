'''
Script for offline simulation of Crazyflie model. It uses model of quadrotor's
dynamic from online program. This model simulates open-loop system (without
position feedback but with feedback control of angles and angular velocities).
This model tuned with respect to telemetry data taken from real flight of CF.
'''

import root as r
import feedback_control
import quad_model
from scipy import integrate
import numpy as np
import matplotlib.pyplot as plt
import sys
import csv

plt.ion()

# Import log file with telemetry data from CF
# Write log file name as parameter
file_name = str(sys.argv[1])
with open(file_name, 'rb') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    records_shape = map(int, reader.next())
    num_rec = records_shape[0]
    data_matrix = np.zeros(records_shape)
    field_names = reader.next()
    for rec in xrange(num_rec):
        data_matrix[rec,:] = map(float, reader.next())

attitude_rate = 500 # Hz
dt1 = 1.0 / attitude_rate
# Matrix which maps pid controllers' outputs into motors' pwm 
M1 = np.array([[1,0,1,1], [1,-1,0,-1], [1,1,-1,1], [1,1,0,-1]])
# Matrix which maps forces produced by motors into torques
M2 = np.array([[0,-r.l,0,r.l], [r.l,0,-r.l,0]])
motors_pwm = np.zeros(4)

# Init pid controllers with coefficients taken from CF firmware
# Yaw and yaw rate pid controllers
yaw_pid_cf = feedback_control.pid(10,1,0.35,1,dt1)
yaw_rate_pid_cf = feedback_control.pid(70,16.7,0,1,dt1)
# Roll and roll rate pid controllers
roll_pid_cf = feedback_control.pid(3.5,2,0,1,dt1)
roll_rate_pid_cf = feedback_control.pid(70,0,0,1,dt1)
# Pitch and pitch rate pid controllers
pitch_pid_cf = feedback_control.pid(3.5,2,0,1,dt1)
pitch_rate_pid_cf = feedback_control.pid(70,0,0,1,dt1)

# Inputs for this model are: thrust set, roll set, pitch set and yaw set
# Postfix '_ti' in var names means "from telemetry interpolated with new dt"
time_t = data_matrix[:, field_names.index('time')]
t_end = time_t[-1]
N = int(round(t_end / dt1))
time_ti = np.interp(np.linspace(0, num_rec - 1, N), np.arange(num_rec), time_t)

# todo: consider to use interpolation from scipy with zero-order hold
roll_set_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('roll_set')])
pitch_set_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('pitch_set')])
yaw_set_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('yaw_set')])
pwm_set = 36950 * np.ones(N)# will be replaced by data from real experiment

x_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('x')])
y_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('y')])

# The main simulation loop
# Initial vector values
y = np.concatenate([r.vel_bi, r.pos_gi, r.angles_gi, r.omega_bi])
y = np.concatenate([[y],[y]])
y_hist = np.zeros((y.shape[1], N))# log of state vector
for n in xrange(N - 1):
    # Calc. force using lookup tables
    rpm = np.interp(pwm_set[n], r.pwm_table, r.rpm_table)
    r.force = np.interp(rpm, r.rpm_table, r.thrust_table)
    # Calc. feedback control
    # Roll
    roll_pid_cf.setpoint = roll_set_ti[n]
    roll_rate_pid_cf.setpoint = roll_pid_cf.evaluate(np.rad2deg(y[1][6]))
    roll_rate_out = roll_rate_pid_cf.evaluate(np.rad2deg(y[1][9]))
    roll_rate_out = np.clip(roll_rate_out, -65536, 65536)
    # Pitch
    pitch_pid_cf.setpoint = pitch_set_ti[n]
    pitch_rate_pid_cf.setpoint = pitch_pid_cf.evaluate(np.rad2deg(y[1][7]))
    pitch_rate_out = pitch_rate_pid_cf.evaluate(np.rad2deg(y[1][10]))
    pitch_rate_out = np.clip(pitch_rate_out, -65536, 65536)
    # Yaw
    yaw_pid_cf.setpoint = yaw_set_ti[n]
    yaw_rate_pid_cf.setpoint = yaw_pid_cf.evaluate(np.rad2deg(y[1][8]))
    yaw_rate_out = yaw_rate_pid_cf.evaluate(np.rad2deg(y[1][11]))
    yaw_rate_out = -np.clip(yaw_rate_out, -65536, 65536)
        
    control_out = np.array([pwm_set[n] / 4.0, roll_rate_out, pitch_rate_out,\
        yaw_rate_out])
        
    # Calc. angular velocities for motors
    motors_pwm = np.dot(M1, control_out)
    motors_rpm = np.interp(motors_pwm, r.pwm_table, r.rpm_table)
    motors_force = np.interp(motors_rpm, r.rpm_table, r.thrust_table) * r.g
    motors_w = 2 * np.pi * motors_rpm / 60.0# rad/sec
    # Calc. moment about z axis
    r.tau_psi = np.dot(np.array([-r.k2,r.k2,-r.k2,r.k2]),pow(motors_w,2))
        
    # Calc. moments about x and y axes
    r.tau_phi = np.dot(M2[0],motors_force)
    r.tau_theta = np.dot(M2[1],motors_force)
        
    # Integrate system of nonlinear ODE
    y = integrate.odeint(quad_model.rhs,y[1], \
        np.array([time_ti[n], time_ti[n + 1]]), rtol=1e-5)
    y_hist[:, n] = y[1]

plt.plot(time_ti[0:-1],np.rad2deg(y_hist[6,0:-1]))
plt.plot(time_ti[0:-1],np.rad2deg(y_hist[7,0:-1]))
plt.plot(time_ti[0:-1],np.rad2deg(y_hist[8,0:-1]))
