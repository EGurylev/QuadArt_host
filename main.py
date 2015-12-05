import root as r
import quad_model
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from control_lib import pid
import math
import numpy as np
from scipy import integrate

# model constants
g = 9.81 # gravity acceleration
M = 0.4 # quadrotor's mass
l = 0.2 # length of arm
Jxx = M * math.pow(l,2); # moment of inertia according to x axis
Jyy = M * math.pow(l,2); # moment of inertia according to y axis
Jzz = 2 * M * math.pow(l,2); # moment of inertia according to z axis

# initial values
PosGi = np.array([0,0,0]) # global position
VelBi = np.array([0,0,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0]) # body angular velocity
y = np.concatenate([VelBi, PosGi, AnglesGi, OmegaBi])
y = np.concatenate([[y],[y]])
# window initialization
app = QtGui.QApplication([])
view = gl.GLViewWidget()
view.show()
grid = gl.GLGridItem()
view.addItem(grid)	
PosInit = 0.5 * np.array([[-l,0,0,l,0],[0,l,-l,0,0],[0,0,0,0,0],[2,2,2,2,2]])
pos = quad_model.AffineTransform(y[1], PosInit)
scatter = pg.opengl.GLScatterPlotItem()
scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
view.addItem(scatter)
Timer = QtCore.QTimer()

# Controller initialization
ZController = pid(0.5,0,0.5,r.dt)
ZController.setpoint = 0.0
PsiController = pid(0.5,0,0.5,r.dt)
PsiController.setpoint = 0

XController = pid(0.2,0,0.1,r.dt)
XController.setpoint = 1
ThetaController = pid(0.1,0,0.1,r.dt)

YController = pid(0.2,0,0.1,r.dt)
YController.setpoint = 2.6
PhiController = pid(0.1,0,0.1,r.dt)

def update():
    global scatter, y, Timer
    if r.t <= r.Tend:
        y = integrate.odeint(quad_model.RHS,y[1],np.array([r.t, r.t + r.dt]))
        r.t = r.t + r.dt
        pos = quad_model.AffineTransform(y[1], PosInit)
        scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
        
        # Feedback control system
        # Z control
        r.F = ZController.evaluate(y[1][5])
        r.F += M * g
        # Cascade control of X
        ThetaController.setpoint = XController.evaluate(y[1][3])# control X
        r.tau_theta = ThetaController.evaluate(y[1][7])# control roll
        # Cascade control of Y
        PhiController.setpoint = -YController.evaluate(y[1][4])# control Y
        r.tau_phi = PhiController.evaluate(y[1][6])# control pitch
        # Yaw control
        r.tau_psi = PsiController.evaluate(y[1][8])
    else:
        Timer.stop()


Timer.timeout.connect(update)
Timer.start(1000 * r.dt)
