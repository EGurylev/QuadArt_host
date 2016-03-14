import root as r
import quad_model
import DetectMarkers
import cf
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.widgets.RawImageWidget import RawImageWidget
import control
import math
import numpy as np
import collections
from scipy import integrate
import sys

class MainUI(QtGui.QWidget):

    def __init__(self, parent=None):
        super(MainUI, self).__init__(parent)
        self.y = np.concatenate([r.VelBi, r.PosGi, r.AnglesGi, r.OmegaBi])
        self.y = np.concatenate([[self.y],[self.y]])

        ## Set custom widgets
        self.view3DW = gl.GLViewWidget()# the main 3D widget
        self.plotW = pg.PlotWidget()# widget for 2D plots
        self.imW = RawImageWidget()# image widget
        # Set size policy for custom widgets
        self.plotW.sizeHint = self.imW.sizeHint = lambda: pg.QtCore.QSize(400, 200)
        self.view3DW.sizeHint = lambda: pg.QtCore.QSize(400, 400)
        self.view3DW.setSizePolicy(self.plotW.sizePolicy())
        self.imW.setSizePolicy(self.plotW.sizePolicy())
        # Create a grid layout to manage the widgets size and position
        self.layout = QtGui.QGridLayout()
        self.setLayout(self.layout)
        # Add widgets to the layout in their proper positions
        self.layout.addWidget(self.view3DW, 0, 0, 2, 1)  # 3D widget goes on left side, spanning 2 rows
        self.layout.addWidget(self.plotW, 0, 1)  # plot goes on upper-right
        self.layout.addWidget(self.imW, 1, 1)  # plot goes on bottom-right
        self.grid = gl.GLGridItem()# grid on 3D widget
        self.view3DW.addItem(self.grid)
        self.Axes = gl.GLLinePlotItem(pos=r.AxesPos, mode='lines', color=r.AxesColor, width=4.0)
        self.view3DW.addItem(self.Axes)
        self.pos = quad_model.AffineTransform(self.y[1], r.PosInit, 'XYZ')
        self.model_sc = pg.opengl.GLScatterPlotItem()
        self.marker_model = gl.GLMeshItem(vertexes=r.verts, faces=r.faces, faceColors=r.colors, smooth=False)
        self.marker_model.setGLOptions('additive')
        self.model_sc.setData(pos=self.pos[0:3,:].T,color=(1,0,0,.3))
        self.exp_sc = pg.opengl.GLScatterPlotItem()
        Color = np.array([[0,1,0,.3], [0,0,1,.3], [0,1,0,.3], [0,1,0,.3]])
        self.exp_sc.setData(pos=self.pos[0:3,:].T,color=Color)
        self.view3DW.addItem(self.model_sc)
        self.view3DW.addItem(self.marker_model)
        self.view3DW.addItem(self.exp_sc)

        # Thread for searching visual marker on quadrotor
        self.improc = DetectMarkers.ImageThread()
        self.connect(self.improc, QtCore.SIGNAL("Sig(PyQt_PyObject, PyQt_PyObject)"), \
            self.im_threadDone, QtCore.Qt.DirectConnection)
        self.improc.start()
        self.Dist = 0
        self.X = 0
        self.Y = 0
        
        # Thread for data exchange with crazyflie
        self.cf_exch = cf.CrazyflieThread()
        self.connect(self.cf_exch, QtCore.SIGNAL("Sig_cf(PyQt_PyObject, PyQt_PyObject, PyQt_PyObject)"), \
            self.cf_threadDone, QtCore.Qt.DirectConnection)
        self.cf_exch.start()
        self.roll_cf = 0
        self.pitch_cf = 0
        self.yaw_cf = 0
        

        self.plot_buffer_y = collections.deque(np.zeros(100), maxlen=100)
        self.plot_buffer_x = collections.deque(np.zeros(100), maxlen=100)
        self.PItem = self.plotW.plot(self.plot_buffer_x, self.plot_buffer_y)
        self.Timer = QtCore.QTimer()
        self.Timer.timeout.connect(self.update)
        self.Timer.start(1000 * r.dt)
        

    def im_threadDone(self, frame, MarkerSize):
        self.imW.setImage(np.rot90(frame))
        self.plot_buffer_y.append(MarkerSize)
        self.plot_buffer_x.append(r.t)
        self.PItem.setData(self.plot_buffer_x, self.plot_buffer_y)
        
        
    def cf_threadDone(self, roll, pitch, yaw):
        self.roll_cf = roll * math.pi / 180.0
        self.pitch_cf = pitch * math.pi / 180.0
        self.yaw_cf = yaw * math.pi / 180.0
    

    def update(self):
        ## Simulated
        self.y = integrate.odeint(quad_model.RHS,self.y[1],np.array([r.t, r.t + r.dt]))
        r.t = r.t + r.dt
        pos = quad_model.AffineTransform(self.y[1], r.PosInit, 'XYZ')
        self.model_sc.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))     
        # Feedback control system
        r.F, r.tau_theta, r.tau_phi, r.tau_psi = control.control_loop(self.y)
        
        ## Measured
        y_m = [0, 0, 0, self.X, self.Dist, self.Y, self.roll_cf, -self.pitch_cf, self.yaw_cf]
        pos = quad_model.AffineTransform(y_m, r.PosInit, 'ZYX')
        initv = np.vstack([r.verts.T, np.ones(8)])
        posv = quad_model.AffineTransform(y_m, initv, 'ZYX')
        self.exp_sc.setData(pos=pos[0:3,:].T)
        self.marker_model.setMeshData(vertexes=posv[0:3,:].T, faces=r.faces, faceColors=r.colors, smooth=False)
        
        
    def closeEvent(self, event):
        self.deleteLater()
        self.Timer.stop()
        self.cf_exch.terminate()
        self.cf_exch._cf.close_link()
        self.improc.cap.release()
        self.improc.terminate()
        

# Start up the main user-interface
app = QtGui.QApplication([])
main_window = MainUI()
main_window.showMaximized()
sys.exit(app.exec_())
