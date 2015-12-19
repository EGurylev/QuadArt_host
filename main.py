import root as r
import quad_model
import DetectMarkers
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.widgets.RawImageWidget import RawImageWidget
import control
import math
import numpy as np
from scipy import integrate

def threadDone(frame):
    imW.setImage(np.rot90(frame))

# Initial values
PosGi = np.array([0,0,0]) # global position
VelBi = np.array([0,0,0]) # body linear velocity
AnglesGi = np.array([0,0,0]) # global angles (Euler)
OmegaBi = np.array([0,0,0]) # body angular velocity
y = np.concatenate([VelBi, PosGi, AnglesGi, OmegaBi])
y = np.concatenate([[y],[y]])
## Window initialization
app = QtGui.QApplication([])

## Define a top-level widget to hold everything
mainW = QtGui.QWidget()

## Set custom widgets
view3DW = gl.GLViewWidget()# the main 3D widget
plotW = pg.PlotWidget()# widget for 2D plots
imW = RawImageWidget()# image widget
# Set size policy for custom widgets
plotW.sizeHint = imW.sizeHint = lambda: pg.QtCore.QSize(400, 200)
view3DW.sizeHint = lambda: pg.QtCore.QSize(400, 400)
view3DW.setSizePolicy(plotW.sizePolicy())
imW.setSizePolicy(plotW.sizePolicy())
# Create a grid layout to manage the widgets size and position
layout = QtGui.QGridLayout()
mainW.setLayout(layout)
# Add widgets to the layout in their proper positions
layout.addWidget(view3DW, 0, 0, 2, 1)  # 3D widget goes on left side, spanning 2 rows
layout.addWidget(plotW, 0, 1)  # plot goes on upper-right
layout.addWidget(imW, 1, 1)  # plot goes on bottom-right
grid = gl.GLGridItem()# grid on 3D widget
view3DW.addItem(grid)

mainW.showMaximized()
	
PosInit = 0.5 * np.array([[-r.l,0,0,r.l,0],[0,r.l,-r.l,0,0],[0,0,0,0,0],[2,2,2,2,2]])
pos = quad_model.AffineTransform(y[1], PosInit)
scatter = pg.opengl.GLScatterPlotItem()
scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
view3DW.addItem(scatter)
Timer = QtCore.QTimer()

# Search visual marker on quadrotor within its own thread
improc = DetectMarkers.ImageThread()
mainW.connect(improc, QtCore.SIGNAL("NdArraySig(PyQt_PyObject)"), threadDone, QtCore.Qt.DirectConnection)
improc.start()
    

def update():
    global scatter, y, Timer

    y = integrate.odeint(quad_model.RHS,y[1],np.array([r.t, r.t + r.dt]))
    r.t = r.t + r.dt
    pos = quad_model.AffineTransform(y[1], PosInit)
    scatter.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))
        
    # Feedback control system
    r.F, r.tau_theta, r.tau_phi, r.tau_psi = control.control_loop(y)


Timer.timeout.connect(update)
Timer.start(1000 * r.dt)
