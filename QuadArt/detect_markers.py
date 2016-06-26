'''
Module for marker searching on image provided by camera.
It uses OpenCV as a base image processing library and it runs
as a separate thread. The core method is mean_shift (do not
confuse with meanShift from OpenCV) which outputs coordinates
of a marker's center. The function from OpenCV uses confidence
map based on color histogram while custom function
operates with binary image. There are two reasons:
    1. Custom mean shift algorithm uses simplified "confidence map" - 
probability 1 for pixel with True value and 0 for False and therefore
histogram calculation and caclulation of confidence map do not needed.
    2. Binary image is a result of finding geometrical shape with
known characteristics. Algorithm searches square shape and convert it
to contour and this contour used as binary image described above.

find_corners calculates coordinates of marker's corners'
using cornerHarris from OpenCV. Tracking algorithm works in two
regimes: 1) full size input image when marker not found and
2) cropped input image based on coordinates of previous successfully
found marker.
    
'''

import root as r
import pypylon
from scipy import signal
from scipy import misc
from pyqtgraph.Qt import QtCore
import numpy as np
import cv2
import time

class image_thread(QtCore.QThread):
    def __init__(self, parent=None):
        super(image_thread, self).__init__(parent)
        # Initial settings
        self.perimeter_prev = 200
        self.marker_size = self.perimeter_prev / 4
        self.area_prev = self.marker_size * self.marker_size
        self.marker_coord = np.array([r.w / 2, r.h / 2], dtype=np.int32)# Center of a marker
        self.win_scale = 2 # scale factor for search window
        self.marker_found_prev = False
        # Init Basler camera
        available_cameras = pypylon.factory.find_devices()
        self.cam = pypylon.factory.create_device(available_cameras[0])
        self.cam.open()
        self.cam.camera_init()
        
        self.marker_found = False
        self.Sig = QtCore.pyqtSignal(np.ndarray, bool, float)
        
        
    def run(self):
        while True:
            self.update()
            self.emit(QtCore.SIGNAL("Sig(PyQt_PyObject, PyQt_PyObject, PyQt_PyObject)"),\
                self.red_frame_c, self.marker_found, r.tvec[2][0])
    
    
    def update(self):
        self.marker_search()
        


    def mean_shift(self, frame, marker_coord):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame_size = gray.shape
        blur = cv2.blur(gray,(13,13))
        frame = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,27,-3)
        _, contours, _ = cv2.findContours( frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        area = []
        perimeter = []
        for cont in contours:
            area.append(cv2.contourArea(cont))
            perimeter.append(cv2.arcLength(cont,1))
            
        area = np.array(area)
        perimeter = np.array(perimeter)
            
        area_diff = abs(area - self.area_prev) / self.area_prev
        perimeter_diff = abs(perimeter - self.perimeter_prev) / self.perimeter_prev
        diff = area_diff + perimeter_diff
                    
        if area.size:
            idx = diff.argmin()
            min_diff = diff[idx]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.frame_c,str(min_diff),(100,500), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(self.frame_c,str(area[idx]),(100,540), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(self.frame_c,str(perimeter[idx]),(100,580), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            if min_diff > 0.2:
                marker_found = False
                cv2.putText(self.frame_c,str(marker_found),(100,620), font, 0.8,(255,255,255),2,cv2.LINE_AA)
                return (marker_found, np.array([0, 0]))
            else:
                marker_found = True
            cv2.putText(self.frame_c,str(marker_found),(100,620), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            c = contours[idx]
            self.area_prev = area[idx]
            self.perimeter_prev = perimeter[idx]
            n = c.shape[0]
            c = c.squeeze()
            c = c.reshape(n * 2, 1)
            y = np.array(c[1::2])
            x = np.array(c[0::2])           
            marker_coord = np.array([int(x.mean()), int(y.mean())])
            marker_found = self.find_corners(x, y, frame_size)
        else:
            return (False, np.array([0, 0]))
                    
        return (marker_found, marker_coord)
        
      
    def find_corners(self, x, y, frame_size):
        corner_coord_temp = np.zeros((4, 2))
        cont_im = np.zeros(frame_size, np.float32)
        cont_im[y,x] = 1
        dst = cv2.cornerHarris(cont_im,15,15,0.1)
        _, dst = cv2.threshold(dst,0.4 * dst.max(),1,0)
        dst = np.uint8(dst)
        _, contours_corn, _ = cv2.findContours( dst.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        font = cv2.FONT_HERSHEY_SIMPLEX
        #cv2.putText(self.frame_c,str(len(contours_corn)),(100,660), font, 0.8,(255,255,255),2,cv2.LINE_AA)
        if len(contours_corn) != 4:
            marker_found = False
            return marker_found
        else:
            marker_found = True
        i = 0
        for cont in contours_corn:
            n = cont.shape[0]
            c = cont.squeeze()
            c = c.reshape(n * 2, 1)
            y = np.array(c[1::2])
            x = np.array(c[0::2])           
            corner_coord_temp[i][:] = np.array([int(x.mean()), int(y.mean())])
            # Reorder corner points for pose estimation algorithm
            if corner_coord_temp[i][0] < frame_size[0] / 2 and corner_coord_temp[i][1] < frame_size[0] / 2:
                r.corner_coord[0][:] = corner_coord_temp[i][:]
            elif corner_coord_temp[i][0] > frame_size[0] / 2 and corner_coord_temp[i][1] < frame_size[0] / 2:
                r.corner_coord[1][:] = corner_coord_temp[i][:]
            elif corner_coord_temp[i][0] < frame_size[0] / 2 and corner_coord_temp[i][1] > frame_size[0] / 2:
                r.corner_coord[2][:] = corner_coord_temp[i][:]
            elif corner_coord_temp[i][0] > frame_size[0] / 2 and corner_coord_temp[i][1] > frame_size[0] / 2:
                r.corner_coord[3][:] = corner_coord_temp[i][:]
            i += 1
        return marker_found    
        


    def track_marker(self):
        self.marker_size = self.perimeter_prev / 4
        x1 = self.marker_coord[0] - int(self.win_scale * self.marker_size / 2)
        x2 = self.marker_coord[0] + int(self.win_scale * self.marker_size / 2)
        y1 = self.marker_coord[1] - int(self.win_scale * self.marker_size / 2)
        y2 = self.marker_coord[1] + int(self.win_scale * self.marker_size / 2)
        if y1 < 0:
            y1 = 0
        if x1 < 0:
            x1 = 0
        if x2 > r.w:
            x2 = r.w
        if y2 > r.h:
            y2 = r.h
        marker_frame = self.frame_c[y1:y2, x1:x2]
               
        self.marker_coord -= np.array([x1, y1])
        self.marker_found, self.marker_coord = self.mean_shift(marker_frame, self.marker_coord)
        self.marker_coord += np.array([x1, y1])
        r.corner_coord += np.array([x1, y1])
    
        if self.marker_found:
            self.frame_c = cv2.circle(self.frame_c,tuple(self.marker_coord.astype(int)), 6, (255,0,0), -1)
            for corn in r.corner_coord:
                self.frame_c = cv2.circle(self.frame_c,tuple(corn.astype(int)), 5, (0,255,0), -1)
            # Draw rectangle for marker reference
            cv2.rectangle(self.frame_c, (x1, y1), (x2, y2), (0,255,0), 1)


    def find_marker(self):
        self.area_prev = self.marker_size * self.marker_size
        self.marker_found, self.marker_coord = self.mean_shift(self.frame_c, self.marker_coord)

    def marker_search(self):
        e1 = cv2.getTickCount()
        time.sleep(0.01)       
        self.frame_c = self.cam.grab_image()
        #if self.marker_found:
             #self.track_marker()
        #else:
             #self.find_marker()
        
        x1, y1 = r.w / 2, r.h / 2
        x2 = x1 + int(self.marker_size)        
        y2 = y1 + int(self.marker_size)
        # Draw rectangle for marker search window
        cv2.rectangle(self.frame_c, (x1, y1), (x2, y2), (0,255,0), 1)
        
        e2 = cv2.getTickCount()
        dtime = 1000 * (e2 - e1)/ cv2.getTickFrequency()
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.frame_c,str(dtime),(100,660), font, 0.8,(255,255,255),2,cv2.LINE_AA)
        
        # Reduce frame in order to show within widget
        red_frame_c = cv2.resize(self.frame_c, (r.w / 2, r.h / 2))
        self.red_frame_c = cv2.flip(red_frame_c, 1)# Mirror flip
        
        #self.red_frame_c = cv2.cvtColor(red_frame_c, cv2.COLOR_BGR2RGB)
        self.marker_found_prev = self.marker_found
