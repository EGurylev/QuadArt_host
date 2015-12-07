import root as r
import quad_model
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import control
import math
import numpy as np
from scipy import integrate

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
PosInit = 0.5 * np.array([[-r.l,0,0,r.l,0],[0,r.l,-r.l,0,0],[0,0,0,0,0],[2,2,2,2,2]])
pos = quad_model.AffineTransform(y[1], PosInit)
scatter = pg.opengl.GLScatterPlotItem()
scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
view.addItem(scatter)
Timer = QtCore.QTimer()

def update():
    global scatter, y, Timer
    if r.t <= r.Tend:
        y = integrate.odeint(quad_model.RHS,y[1],np.array([r.t, r.t + r.dt]))
        r.t = r.t + r.dt
        pos = quad_model.AffineTransform(y[1], PosInit)
        scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
        
        # Feedback control system
        r.F, r.tau_theta, r.tau_phi, r.tau_psi = control.control_loop(y)
    else:
        Timer.stop()


Timer.timeout.connect(update)
Timer.start(1000 * r.dt)
