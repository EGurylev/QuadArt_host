'''
Script for offline simulation of Crazyflie model. It uses model of quadrotor's
dynamic from online program. This model simulates closed-loop system (outer loop which
controls position and inner loop which controls angles and anfular speeds).
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

# frequency of control loops within cf (angles and angular speeds)
attitude_rate = 500 # Hz
dt1 = 1.0 / attitude_rate
dt2 = r.dt # sample time for outer control loop within PC (position)
delay = 2 * int(dt2 / dt1) # delay of outer control loop (in terms of dt2)

# Calc. rpm and pwm output for equilibrium point   
rpm_eq = np.interp(r.mass, r.thrust_table, r.rpm_table)
thrust_eq = np.interp(rpm_eq, r.rpm_table, r.pwm_table)

# Matrix which maps pid controllers' outputs into motors' pwm 
M1 = np.array([[1,0,1,1], [1,-1,0,-1], [1,1,-1,1], [1,1,0,-1]])
# Matrix which maps forces produced by motors into torques
M2 = np.array([[0,-r.l,0,r.l], [r.l,0,-r.l,0]])
motors_pwm = np.zeros(4)

# Init inner loop pid controllers with coefficients taken from CF firmware
# Yaw and yaw rate pid controllers
yaw_pid_cf = feedback_control.pid(10,1,0.35,1,dt1)
yaw_rate_pid_cf = feedback_control.pid(70,16.7,0,1,dt1)
# Roll and roll rate pid controllers
roll_pid_cf = feedback_control.pid(3.5,2,0,1,dt1)
roll_rate_pid_cf = feedback_control.pid(70,0,0,1,dt1)
# Pitch and pitch rate pid controllers
pitch_pid_cf = feedback_control.pid(3.5,2,0,1,dt1)
pitch_rate_pid_cf = feedback_control.pid(70,0,0,1,dt1)

# Init outer loop pid controllers
z_pid_cf = feedback_control.pid(80,30,90,0.35,1/30.0, upper=7000, lower=-7000, sat_flag=True)
z_pid_cf.setpoint = 0.0
x_pid_cf = feedback_control.pid(0.2,0.04,0.22,0.15,1/30.0, upper=20, lower=-20, sat_flag=True)
x_pid_cf.setpoint = 0.0
y_pid_cf = feedback_control.pid(0.2,0.06,0.22,0.15,1/30.0, upper=20, lower=-20, sat_flag=True)
y_pid_cf.setpoint = 80.0

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
roll_cf_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('roll_cf')])
pitch_cf_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('pitch_cf')])
yaw_cf_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('yaw_cf')])
marker_found = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('marker_found')])
thrust_cf_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('thrust_cf')])
thrust_set_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('thrust_set')]) + r.thrust_eq

x_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('x')]) / 1e2# cm to m
y_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('y')]) / 1e2# cm to m
z_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('z')]) / 1e2# cm to m
vbat_ti = np.interp(time_ti, time_t, \
    data_matrix[:, field_names.index('vbat')])
    
# flag which indicates that connection with cf has been established   
cf_connected = vbat_ti != 0.0

# The main simulation loop
# Initial vector values
y = np.zeros(12)
y[4] = r.tvec[2] / 1e2
y = np.concatenate([[y],[y]])
y_hist = np.zeros((y.shape[1], N))# log of state vector
thrust_set = 0# initial value
pitch_set = 0# initial value
roll_set = 0# initial value
for n in xrange(N - 1):
    if marker_found[n] and not marker_found[n - int(dt2 / dt1)]:
        # Initialize position and angles
        y[1][6:9] = np.deg2rad(np.array([roll_cf_ti[n], pitch_cf_ti[n], \
            yaw_cf_ti[n]]))
        y[1][3:6] = np.array([x_ti[n], y_ti[n], z_ti[n]])
            
    # Calc. outer loop feedback control. It runs slower than inner loop.
    if n % int(dt2 / dt1) == 0:
        z_meas = (y_hist[5, n - delay] + (np.random.randn() - np.random.randn()) * 1e-5) * 1e2
        x_meas = (y_hist[3, n - delay] + (np.random.randn() - np.random.randn()) * 1e-5) * 1e2
        y_meas = (y_hist[4, n - delay] + (np.random.randn() - np.random.randn()) * 1e-4) * 1e2
        thrust_set = z_pid_cf.evaluate(z_meas) + thrust_eq
        pitch_set = x_pid_cf.evaluate(x_meas)
        roll_set = -y_pid_cf.evaluate(y_meas)
        
    # Calc. inner loop feedback control
    # Roll
    roll_pid_cf.setpoint = roll_set
    roll_rate_pid_cf.setpoint = roll_pid_cf.evaluate(np.rad2deg(y[1][6]))
    roll_rate_out = roll_rate_pid_cf.evaluate(np.rad2deg(y[1][9]))
    roll_rate_out = np.clip(roll_rate_out, -65536, 65536)
    # Pitch
    pitch_pid_cf.setpoint = pitch_set
    pitch_rate_pid_cf.setpoint = pitch_pid_cf.evaluate(np.rad2deg(y[1][7]))
    pitch_rate_out = pitch_rate_pid_cf.evaluate(np.rad2deg(y[1][10]))
    pitch_rate_out = np.clip(pitch_rate_out, -65536, 65536)
    # Yaw
    yaw_pid_cf.setpoint = yaw_set_ti[n]
    yaw_rate_pid_cf.setpoint = yaw_pid_cf.evaluate(np.rad2deg(y[1][8]))
    yaw_rate_out = yaw_rate_pid_cf.evaluate(np.rad2deg(y[1][11]))
    yaw_rate_out = -np.clip(yaw_rate_out, -65536, 65536)
            
    control_out = np.array([thrust_set, roll_rate_out, pitch_rate_out,\
        yaw_rate_out])
            
    # Outer loop feedback control works only when connection with CF is established
    if cf_connected[n]:
        # Calc. angular velocities for motors
        motors_pwm = np.dot(M1, control_out)
        motors_rpm = np.interp(motors_pwm, r.pwm_table, r.rpm_table)
        motors_force = np.interp(motors_rpm, r.rpm_table, r.thrust_table) * r.g / 4.0
        motors_w = 2 * np.pi * motors_rpm / 60.0# rad/sec
        # Calc. moment about z axis
        r.tau_psi = np.dot(np.array([-r.k2,r.k2,-r.k2,r.k2]),pow(motors_w,2))

        # Calc. moments about x and y axes
        r.tau_phi = np.dot(M2[0],motors_force)
        r.tau_theta = np.dot(M2[1],motors_force)
        r.force = motors_force.sum()
    else:
        r.force = r.mass * r.g # valid before launch
        r.tau_psi = 0
        r.tau_phi = 0
        r.tau_theta = 0
    # Calc. force using voltage-thrust plynomial fit (alternative)
    #r.force = np.polyval(r.thrust_volt_fit, vbat_ti[n] * thrust_set / 65536.0) * 1e-3 * r.g
    # Integrate system of nonlinear ODEs
    y = integrate.odeint(quad_model.rhs,y[1], \
        np.array([time_ti[n], time_ti[n + 1]]), rtol=1e-5)
    y_hist[:, n] = y[1]

plt.plot(time_ti[0:-1],z_ti[0:-1])
plt.plot(time_ti[0:-1],y_hist[5,0:-1])

plt.plot(time_ti[0:-1],x_ti[0:-1])
plt.plot(time_ti[0:-1],y_hist[3,0:-1])

plt.plot(time_ti[0:-1],y_ti[0:-1])
plt.plot(time_ti[0:-1],y_hist[4,0:-1])
