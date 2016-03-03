import root as r
from scipy import signal
from scipy import misc
from pyqtgraph.Qt import QtCore
import numpy as np
import math
import cv2


class ImageThread(QtCore.QThread):
    def __init__(self, parent=None):
        super(ImageThread, self).__init__(parent)
        #Initial settings
        self.W, self.H = 1280, 720
        self.PerimeterPrev = 400
        self.MarkerSize = self.PerimeterPrev / 4
        self.AreaPrev = self.MarkerSize * self.MarkerSize
        self.MarkerCoord = np.array([self.W / 2, self.H / 2], dtype=np.int32)
        self.WinScale = 1.8 # scale factor for search window
        
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.MarkerFound = 0
        self.Sig = QtCore.pyqtSignal(np.ndarray, list, float)
        
        
    def run(self):
        while True:
            self.update()
            self.emit(QtCore.SIGNAL("Sig(PyQt_PyObject, PyQt_PyObject)"), self.RedframeC, self.MarkerSize)
    
    
    def update(self):
        self.marker_search()
        


    def MeanShift(self, frame, MarkerCoord):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,27,-3)
        _, contours, _ = cv2.findContours( frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Area = []
        Perimeter = []
        for cont in contours:
            A = cv2.contourArea(cont)
            P = cv2.arcLength(cont,1)
            Area.append(A)
            Perimeter.append(P)
            
        Area = np.array(Area)
        Perimeter = np.array(Perimeter)
            
        AreaDiff = abs(Area - self.AreaPrev) / self.AreaPrev
        PerimeterDiff = abs(Perimeter - self.PerimeterPrev) / self.PerimeterPrev
        Diff = AreaDiff + PerimeterDiff
                    
        if Area.size:
            idx = Diff.argmin()
            MinDiff = Diff[idx]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(self.frameC,str(MinDiff),(100,500), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(self.frameC,str(Area[idx]),(100,540), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            cv2.putText(self.frameC,str(Perimeter[idx]),(100,580), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            if MinDiff > 0.2:
                MarkerFound = False
                cv2.putText(self.frameC,str(MarkerFound),(100,620), font, 0.8,(255,255,255),2,cv2.LINE_AA)
                return (MarkerFound, np.array([0, 0]))
            else:
                MarkerFound = True
            cv2.putText(self.frameC,str(MarkerFound),(100,620), font, 0.8,(255,255,255),2,cv2.LINE_AA)
            C = contours[idx]
            self.AreaPrev = Area[idx]
            self.PerimeterPrev = Perimeter[idx]
            N = C.shape[0]
            C = C.squeeze()
            C = C.reshape(N * 2, 1)
            Y = np.array(C[1::2])
            X = np.array(C[0::2])           
            MarkerCoord = np.array([int(X.mean()), int(Y.mean())])
            
        else:
            return (False, np.array([0, 0]))
                    
        return (MarkerFound, MarkerCoord)


    def TestMarker(self):
        Lengths = np.zeros(4)
        for n in xrange(4):
            if n == 3:
                Lengths[n] = dist(self.MarkerCoord[3], self.MarkerCoord[0])
            else:
                Lengths[n] = dist(self.MarkerCoord[n], self.MarkerCoord[n + 1])

        self.MeanLength = Lengths.mean()
        
        if Lengths.std() / self.MeanLength < 0.1:
            self.MarkerFound = 1
        else:
            self.MarkerFound = 0


    def TrackMarker(self):
        self.MarkerSize = self.PerimeterPrev / 4
        X1 = self.MarkerCoord[0] - int(self.WinScale * self.MarkerSize / 2)
        X2 = self.MarkerCoord[0] + int(self.WinScale * self.MarkerSize / 2)
        Y1 = self.MarkerCoord[1] - int(self.WinScale * self.MarkerSize / 2)
        Y2 = self.MarkerCoord[1] + int(self.WinScale * self.MarkerSize / 2)
        if Y1 < 0:
            Y1 = 0
        if X1 < 0:
            X1 = 0
        if X2 > self.W:
            X2 = self.W
        if Y2 > self.H:
            Y2 = self.H
        MarkerFrame = self.frameC[Y1:Y2, X1:X2]
               
        self.MarkerCoord -= np.array([X1, Y1])
        self.MarkerFound, self.MarkerCoord = self.MeanShift(MarkerFrame, self.MarkerCoord)
        self.MarkerCoord += np.array([X1, Y1])
    
        self.frameC = cv2.circle(self.frameC,tuple(self.MarkerCoord.astype(int)), 6, (255,0,), -1)
        # Draw rectangle for marker reference
        cv2.rectangle(self.frameC, (X1, Y1), (X2, Y2), (0,255,0), 1)


    def FindMarker(self):
        self.AreaPrev = self.MarkerSize * self.MarkerSize
        self.MarkerFound, self.MarkerCoord = self.MeanShift(self.frameC, self.MarkerCoord)

    def marker_search(self):
        ret, self.frameC = self.cap.read()
        
        if self.MarkerFound:
             self.TrackMarker()
             #MeanAreaHist.append(math.pow(self.MeanLength / 9.42, 2))
             #RadiusHist.append(self.MarkerRadius)
        else:
             self.FindMarker()
        
        x1, y1 = self.W / 2, self.H / 2
        x2 = x1 + int(self.MarkerSize)        
        y2 = y1 + int(self.MarkerSize)
        # Draw rectangle for marker search window
        cv2.rectangle(self.frameC, (x1, y1), (x2, y2), (0,255,0), 1)
        
        # Reduce frame in order to show within widget
        RedframeC = cv2.resize(self.frameC, (self.W / 2, self.H / 2))
        RedframeC = cv2.flip(RedframeC, 1)# Mirror flip
        self.RedframeC = cv2.cvtColor(RedframeC, cv2.COLOR_BGR2RGB)
