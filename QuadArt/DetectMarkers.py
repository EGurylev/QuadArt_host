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
        self.DebugFrame = []
        self.MarkerSize = self.PerimeterPrev / 4
        self.AreaPrev = self.MarkerSize * self.MarkerSize
        self.MarkerCoord = np.array([self.W / 2, self.H / 2], dtype=np.int32)# Center of a marker
        self.CornerCoord = np.zeros([4,2])
        self.WinScale = 1.8 # scale factor for search window
        self.MarkerFoundPrev = False
        self.FalseReason = 0
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.MarkerFound = 0
        self.Sig = QtCore.pyqtSignal(np.ndarray, list, int)
        
        
    def run(self):
        while True:
            self.update()
            self.emit(QtCore.SIGNAL("Sig(PyQt_PyObject, PyQt_PyObject)"), self.RedframeC, self.FalseReason)
    
    
    def update(self):
        self.marker_search()
        


    def MeanShift(self, frame, MarkerCoord):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        DebugFrame = gray
        FrameSize = gray.shape
        blur = cv2.blur(gray,(13,13))
        frame = cv2.adaptiveThreshold(blur,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,27,-3)
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
                self.FalseReason = 1
                return (MarkerFound, np.array([0, 0]))
            else:
                MarkerFound = True
                self.FalseReason = 0
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
            MarkerFound = self.FindCorners(X, Y, FrameSize)
        else:
            return (False, np.array([0, 0]))
                    
        return (MarkerFound, MarkerCoord)
        
      
    def FindCorners(self, X, Y, FrameSize):
        ContIm = np.zeros(FrameSize, np.float32)
        ContIm[Y,X] = 1
        dst = cv2.cornerHarris(ContIm,15,15,0.1)
        _, dst = cv2.threshold(dst,0.4 * dst.max(),1,0)
        dst = np.uint8(dst)
        _, contoursCorn, _ = cv2.findContours( dst.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.frameC,str(len(contoursCorn)),(100,660), font, 0.8,(255,255,255),2,cv2.LINE_AA)
        if len(contoursCorn) != 4:
            MarkerFound = False
            self.FalseReason = 2
            return MarkerFound
        else:
            MarkerFound = True
        i = 0
        for cont in contoursCorn:
            N = cont.shape[0]
            C = cont.squeeze()
            C = C.reshape(N * 2, 1)
            Y = np.array(C[1::2])
            X = np.array(C[0::2])           
            self.CornerCoord[i][:] = np.array([int(X.mean()), int(Y.mean())])
            i += 1
        return MarkerFound    
        


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
        self.CornerCoord += np.array([X1, Y1])
        
        if (self.MarkerFoundPrev) and (not self.MarkerFound) and (self.FalseReason == 2):
            self.DebugFrame = MarkerFrame.copy()
    
        if self.MarkerFound:
            self.frameC = cv2.circle(self.frameC,tuple(self.MarkerCoord.astype(int)), 6, (255,0,0), -1)
            for corn in self.CornerCoord:
                self.frameC = cv2.circle(self.frameC,tuple(corn.astype(int)), 5, (0,255,0), -1)
            # Draw rectangle for marker reference
            cv2.rectangle(self.frameC, (X1, Y1), (X2, Y2), (0,255,0), 1)


    def FindMarker(self):
        self.AreaPrev = self.MarkerSize * self.MarkerSize
        self.MarkerFound, self.MarkerCoord = self.MeanShift(self.frameC, self.MarkerCoord)

    def marker_search(self):
        ret, self.frameC = self.cap.read()
        e1 = cv2.getTickCount()
            
        if self.MarkerFound:
             self.TrackMarker()
        else:
             self.FindMarker()
        e2 = cv2.getTickCount()
        time = 1000 * (e2 - e1)/ cv2.getTickFrequency()#ms       
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.frameC,str(time),(100,450), font, 0.8,(255,255,255),2,cv2.LINE_AA)
        x1, y1 = self.W / 2, self.H / 2
        x2 = x1 + int(self.MarkerSize)        
        y2 = y1 + int(self.MarkerSize)
        # Draw rectangle for marker search window
        cv2.rectangle(self.frameC, (x1, y1), (x2, y2), (0,255,0), 1)
        
        # Reduce frame in order to show within widget
        RedframeC = cv2.resize(self.frameC, (self.W / 2, self.H / 2))
        RedframeC = cv2.flip(RedframeC, 1)# Mirror flip
        self.RedframeC = cv2.cvtColor(RedframeC, cv2.COLOR_BGR2RGB)
        self.MarkerFoundPrev = self.MarkerFound
