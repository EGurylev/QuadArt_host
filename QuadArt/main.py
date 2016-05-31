import root as r
import quad_model
import detect_markers
import pose_estim
import cf
import post
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.widgets.RawImageWidget import RawImageWidget
import feedback_control
import numpy as np
import collections
from scipy import integrate
import sys
import cv2

class main_ui(QtGui.QWidget):

    def __init__(self, parent=None):
        super(main_ui, self).__init__(parent)
        self.y = np.concatenate([r.vel_bi, r.pos_gi, r.angles_gi, r.omega_bi])
        self.y = np.concatenate([[self.y],[self.y]])

        ## Set custom widgets
        self.view_3d_w = gl.GLViewWidget()# the main 3D widget
        self.plot_w = pg.PlotWidget()# widget for 2D plots
        self.im_w = RawImageWidget()# image widget
        # Set size policy for custom widgets
        self.plot_w.sizeHint = self.im_w.sizeHint = lambda: pg.QtCore.QSize(400, 200)
        self.view_3d_w.sizeHint = lambda: pg.QtCore.QSize(400, 400)
        self.view_3d_w.setSizePolicy(self.plot_w.sizePolicy())
        self.im_w.setSizePolicy(self.plot_w.sizePolicy())
        # Create a grid layout to manage the widgets size and position
        self.layout = QtGui.QGridLayout()
        self.setLayout(self.layout)
        # Add widgets to the layout in their proper positions
        self.layout.addWidget(self.view_3d_w, 0, 0, 2, 1)  # 3D widget goes on left side, spanning 2 rows
        self.layout.addWidget(self.plot_w, 0, 1)  # plot goes on upper-right
        self.layout.addWidget(self.im_w, 1, 1)  # plot goes on bottom-right
        self.grid = gl.GLGridItem()# grid on 3D widget
        self.view_3d_w.addItem(self.grid)
        self.axes = gl.GLLinePlotItem(pos=r.axes_pos, mode='lines', color=r.axes_color, width=4.0)
        self.view_3d_w.addItem(self.axes)
        self.pos = quad_model.homog_transform(self.y[1], r.pos_init, 'XYZ')
        self.model_sc = pg.opengl.GLScatterPlotItem()
        self.marker_model = gl.GLMeshItem(vertexes=r.verts, faces=r.faces, faceColors=r.colors, smooth=False)
        self.marker_model.setGLOptions('additive')
        self.model_sc.setData(pos=self.pos[0:3,:].T,color=(1,0,0,.3))
        self.exp_sc = pg.opengl.GLScatterPlotItem()
        color = np.array([[0,1,0,.3], [0,0,1,.3], [0,1,0,.3], [0,1,0,.3]])
        self.exp_sc.setData(pos=self.pos[0:3,:].T,color=color)
        self.view_3d_w.addItem(self.model_sc)
        self.view_3d_w.addItem(self.marker_model)
        self.view_3d_w.addItem(self.exp_sc)
        self.view_3d_w.setCameraPosition(distance=130, azimuth=90, elevation=10)

        # Thread for searching visual marker on quadrotor
        self.improc = detect_markers.image_thread()
        self.connect(self.improc, QtCore.SIGNAL("Sig(PyQt_PyObject, PyQt_PyObject, PyQt_PyObject)"), \
            self.im_thread_done, QtCore.Qt.DirectConnection)
        self.improc.start()
        
        # Thread for data exchange with crazyflie
        self.cf_exch = cf.crazyflie_thread()
        self.connect(self.cf_exch, QtCore.SIGNAL("Sig_cf(PyQt_PyObject,PyQt_PyObject,\
            PyQt_PyObject,PyQt_PyObject,PyQt_PyObject)"), \
            self.cf_thread_done, QtCore.Qt.DirectConnection)
        self.cf_exch.start()

        self.plot_buffer_y = collections.deque(np.zeros(100), maxlen=100)
        self.plot_buffer_x = collections.deque(np.zeros(100), maxlen=100)
        
        # Initialize deques for logging data
        self.log = {'time': collections.deque(),
                    'thrust_set': collections.deque(),
                    'thrust_cf': collections.deque(),
                    'roll_cf': collections.deque(),
                    'pitch_cf': collections.deque(),
                    'yaw_cf': collections.deque(),
                    'roll_e': collections.deque(),
                    'pitch_e': collections.deque(),
                    'yaw_e': collections.deque(),
                    'roll_set': collections.deque(),
                    'pitch_set': collections.deque(),
                    'yaw_set': collections.deque(),
                    'x': collections.deque(),
                    'y': collections.deque(),
                    'z': collections.deque(),
                    'marker_found': collections.deque(),
                    'vbat': collections.deque()}
        
        self.plot_item = self.plot_w.plot(self.plot_buffer_x, self.plot_buffer_y)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000 * r.dt)
        

    def im_thread_done(self, frame, marker_found, debug_plot):
        r.marker_found = marker_found
        if r.marker_found:
            pose_estim.calc_pose()
        self.im_w.setImage(np.rot90(frame))
        self.plot_buffer_y.append(r.tvec[2][0])
        self.plot_buffer_x.append(r.t)
        self.plot_item.setData(self.plot_buffer_x, self.plot_buffer_y)
        
        
    def cf_thread_done(self, roll, pitch, yaw, thrust, vbat):
        r.roll_cf = np.deg2rad(roll)
        r.pitch_cf = np.deg2rad(pitch)
        r.yaw_cf = np.deg2rad(yaw)
        r.thrust_cf = thrust
        r.vbat = vbat
    

    def update(self):
        ## Simulated
        self.y = integrate.odeint(quad_model.rhs,self.y[1],np.array([r.t, r.t + r.dt]))
        r.t = r.t + r.dt
        pos = quad_model.homog_transform(self.y[1], r.pos_init, 'XYZ')
        self.model_sc.setData(pos=pos[0:3,:].T,color=(1,0,0,.3))     
        # Feedback control system
        r.force, r.tau_theta, r.tau_phi, r.tau_psi = feedback_control.control_loop(self.y)
        
        ## Measured
        y_m = [0, 0, 0, r.tvec[0][0], r.tvec[2][0], -r.tvec[1][0], r.roll_cf, -r.pitch_cf, r.yaw_cf]
        #y_m = [0, 0, 0, r.tvec[0][0], r.tvec[2][0], -r.tvec[1][0], r.rvec[0][0], r.rvec[2][0], r.rvec[1][0]]
        pos = quad_model.homog_transform(y_m, r.pos_init, 'ZYX')
        initv = np.vstack([r.verts.T, np.ones(12)])
        posv = quad_model.homog_transform(y_m, initv, 'ZYX')
        self.exp_sc.setData(pos=pos[0:3,:].T)
        self.marker_model.setMeshData(vertexes=posv[0:3,:].T, faces=r.faces, faceColors=r.colors, smooth=False)
        
        ## CF control
        ## Pose control of CF
        if abs(r.tvec[1][0]) < 150:# todo: reconsider this condition
            r.thrust_set = feedback_control.z_controller_cf.evaluate(-r.tvec[1][0])
            r.pitch_set = feedback_control.x_controller_cf.evaluate(r.tvec[0][0])
            r.roll_set = -feedback_control.y_controller_cf.evaluate(r.tvec[2][0])
        
        ## Log data
        self.log['time'].append(r.t)
        self.log['thrust_set'].append(r.thrust_set)
        self.log['thrust_cf'].append(r.thrust_cf)
        self.log['roll_cf'].append(r.roll_cf)
        self.log['pitch_cf'].append(r.pitch_cf)
        self.log['yaw_cf'].append(r.yaw_cf)
        self.log['roll_e'].append(r.roll_e)
        self.log['pitch_e'].append(r.pitch_e)
        self.log['yaw_e'].append(r.yaw_e)
        self.log['pitch_set'].append(r.pitch_set)
        self.log['roll_set'].append(r.roll_set)
        self.log['yaw_set'].append(r.yaw_set)
        self.log['marker_found'].append(r.marker_found)
        self.log['vbat'].append(r.vbat)
        if abs(r.tvec[0][0]) < 150:# todo: reconsider this condition
            self.log['x'].append(r.tvec[0][0])
            self.log['y'].append(r.tvec[2][0])
            self.log['z'].append(-r.tvec[1][0])
        else:
            self.log['x'].append(150)
            self.log['y'].append(150)
            self.log['z'].append(150)
        
    def closeEvent(self, event):
        self.deleteLater()
        self.timer.stop()
        self.cf_exch.terminate()
        self.cf_exch._cf.close_link()
        self.improc.cap.release()
        self.improc.terminate()
        post.write_log(self.log)
        

# Start up the main user-interface
app = QtGui.QApplication([])
main_window = main_ui()
main_window.showMaximized()
sys.exit(app.exec_())
