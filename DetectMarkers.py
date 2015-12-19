import root as r
from scipy import signal
from scipy import misc
from pyqtgraph.Qt import QtCore
import numpy as np
import math
import cv2

def dist(p1, p2):
    return math.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))
    
def isLine(p1, p2, MaxLength, MinLength):
    # Construct line with length in specific range
    p1 = p1.astype(float)
    p2 = p2.astype(float)
   
    # Calculate length of this line
    l = dist(p1, p2)
    if l < MaxLength and l > MinLength:
        return l
    else:
        return 0


def isCorner(LengthList, AreaList, CentersList):
    # Test given tuple of lines to be perpendicular to each 
    #other and have equal lengths and equal areas of their
    # "points" (blob clusters).

    Tol = 0.1#tolerance band

    # Are the adjacent lines perpendicular to each other?
    V1 = [CentersList[1][0]-CentersList[0][0], CentersList[1][1]-CentersList[0][1]]
    V2 = [CentersList[3][0]-CentersList[2][0], CentersList[3][1]-CentersList[2][1]]
    temp = np.dot(V1,V2) / (LengthList[0] * LengthList[1])
    if temp > 1:
        temp = 1
    elif temp < -1:
        temp = -1
    alpha = math.acos(temp)
    dA  = abs(math.pi/2 - abs(alpha))
    
    # Do the lines have equal areas of end "points"
    dArea = AreaList.std() / AreaList.mean()

    # Are the lines have equal length?
    dL = abs(LengthList[0] - LengthList[1]) / ((abs(LengthList[0]) + abs(LengthList[1])) / 2)

    if (dL < Tol) and (dArea < Tol) and (dA < Tol):
        return 1
    else:
        return 0


class ImageThread(QtCore.QThread):
    def __init__(self, parent=None):
        super(ImageThread, self).__init__(parent)
        #Initial settings
        self.MeanArea = 65.0
        self.MeanLength = 80.0
        self.MarkerCoord = np.zeros((4,2), dtype=np.int32)
        self.WinScale = 1.5 # scale factor for search window
        self.W, self.H = 1280, 720
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.H)
        self.MarkerFound = 0
        self.NdArraySig = QtCore.pyqtSignal(np.ndarray)
        
        
    def run(self):
        while True:
            self.update()
            self.emit(QtCore.SIGNAL("NdArraySig(PyQt_PyObject)"), self.RedframeC)
    
    
    def update(self):
        self.marker_search()
        


    def MeanShift(self, frame, MarkerCoord):
        # Search with fixed number of steps
        NumOfMeanShiftSteps = 2
        WindowSize = self.MeanLength
        for i in xrange(NumOfMeanShiftSteps):
            XWindowLeft = MarkerCoord[1] - WindowSize / 2
            XWindowRigth = MarkerCoord[1] + WindowSize / 2
            YWindowDown = MarkerCoord[0] - WindowSize / 2
            YWindowUp = MarkerCoord[0] + WindowSize / 2
            frameW = frame[XWindowLeft:XWindowRigth,YWindowDown:YWindowUp]
            _, contours, _ = cv2.findContours( frameW.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            Area = []
            for cont in contours:
                Area.append(cv2.contourArea(cont))
            Area = np.array(Area)
            Diff = abs(Area - self.MeanArea)
            if Area.size:
                idx = Diff.argmin()
                C = contours[idx]
                N = C.shape[0]
                C = C.squeeze()
                C = C.reshape(N * 2, 1)
                Y = np.array(C[1::2])
                X = np.array(C[0::2])           
                MarkerCoord += np.array([X.mean().round(), Y.mean().round()]) - WindowSize / 2
            else:
                return np.array([0, 0])
                
        return MarkerCoord


    def TestMarker(self):
        Lengths = np.zeros(4)
        for n in xrange(4):
            if n == 3:
                Lengths[n] = dist(self.MarkerCoord[3], self.MarkerCoord[0])
            else:
                Lengths[n] = dist(self.MarkerCoord[n], self.MarkerCoord[n + 1])

        self.LengthMean = Lengths.mean()
        
        if Lengths.std() / self.LengthMean < 0.1:
            self.MarkerFound = 1
        else:
            self.MarkerFound = 0


    def TrackMarker(self):
        MarkerCenter = [int(round(np.mean([self.MarkerCoord[0][0], self.MarkerCoord[1][0], self.MarkerCoord[2][0], \
            self.MarkerCoord[3][0]]))), int(round(np.mean([self.MarkerCoord[0][1], self.MarkerCoord[1][1], \
            self.MarkerCoord[2][1], self.MarkerCoord[3][1]])))]
        X1 = MarkerCenter[0] - int(self.WinScale * self.MeanLength)
        X2 = MarkerCenter[0] + int(self.WinScale * self.MeanLength)
        Y1 = MarkerCenter[1] - int(self.WinScale * self.MeanLength)
        Y2 = MarkerCenter[1] + int(self.WinScale * self.MeanLength)
        MarkerFrame = self.frameC[Y1:Y2, X1:X2]
        cv2.rectangle(self.frameC, (X1, Y1), (X2, Y2), (255,255,255), 1)
        frame = cv2.cvtColor(MarkerFrame, cv2.COLOR_BGR2GRAY)
        frame = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,13,-3)
                
        self.MarkerCoord -= np.array([X1, Y1])
        for m in xrange(4):
            self.MarkerCoord[m] = self.MeanShift(frame, self.MarkerCoord[m])
            if not self.MarkerCoord[m].any():
                self.MarkerFound = 0
                break
                
        self.MarkerCoord += np.array([X1, Y1])
        for m in xrange(4):
            self.frameC = cv2.circle(self.frameC,tuple(self.MarkerCoord[m].astype(int)), 5, (0,255,255), -1)

        self.TestMarker()
        self.MeanArea = math.pow(self.MeanLength / 10, 2)#based on marker geometry
        Shift = self.MeanLength / 2#half of window size
        Shift = int(Shift)
        for Coord in self.MarkerCoord:
            cv2.rectangle(self.frameC, (Coord[0] - Shift, Coord[1] - Shift), \
                (Coord[0] + Shift, Coord[1] + Shift), (0,0,255), 1)


    def FindMarker(self):
        dArea, dLength = 15.0, 10.0
        MinArea, MaxArea = self.MeanArea - dArea, self.MeanArea + dArea
        MinLength, MaxLength = self.MeanLength - dLength, self.MeanLength + dLength
        frame = cv2.cvtColor(self.frameC, cv2.COLOR_BGR2GRAY)
        frame = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,13,-3)
        _, contours0, _ = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            
        # Choose blob clusters only in specified area region
        Clusters = []
        for k in xrange(len(contours0)):
            if cv2.contourArea(contours0[k]) > MinArea and cv2.contourArea(contours0[k]) < MaxArea:
                Clusters.append(contours0[k])
                    
        Clusters = np.array(Clusters)
        ClusterCoord = np.zeros([Clusters.shape[0], 2])
        cv2.fillPoly(self.frameC, pts = Clusters, color=(0,0,255))
            
        # Find clusters' centres
        for i in xrange(Clusters.shape[0]):
                    ClusterCoord[i] = [Clusters[i][:,0][:,0].mean().round(), Clusters[i][:,0][:,1].mean().round()]

        ClusterCoord = ClusterCoord.astype(int)
        Lines = []
        Length = []
        Corners = []

        # Construct line objects
        for p1 in xrange(ClusterCoord.shape[0]):
            for p2 in xrange(p1 + 1,ClusterCoord.shape[0]):
                if p1 == p2:
                    continue
                else:
                    L = isLine(ClusterCoord[p1,:],ClusterCoord[p2,:], MaxLength, MinLength)
                    if L:
                        Length.append(L)
                        Lines.append(sorted([p1, p2]))
                        self.frameC = cv2.line(self.frameC,tuple(ClusterCoord[p1,:]),\
                            tuple(ClusterCoord[p2,:]),(255,0,0),1)

        # Test lines to be perpendicular and equal
        for l1 in xrange(len(Lines)):
            for l2 in xrange(l1 + 1,len(Lines)):
                if (l1 == l2):
                    continue
                else:
                    LengthList = [Length[l1], Length[l2]]
                    AreaList = np.array([len(Clusters[Lines[l1]][0]), len(Clusters[Lines[l1]][1]),\
                        len(Clusters[Lines[l2]][0]), len(Clusters[Lines[l2]][1])])
                    CentersList = [ClusterCoord[Lines[l1]][0], ClusterCoord[Lines[l1]][1],\
                        ClusterCoord[Lines[l2]][0], ClusterCoord[Lines[l2]][1]]
                    if isCorner(LengthList, AreaList, CentersList):
                        Corners.append(sorted([l1, l2]))
            
        Corners = np.array(Corners)
        Area = np.zeros(4)
        for s1 in xrange(len(Corners)):
            for s2 in xrange(len(Corners)):
                P1 = np.sort(np.reshape([Lines[Corners[s1][0]], Lines[Corners[s1][1]]],4))
                P2 = np.sort(np.reshape([Lines[Corners[s2][0]], Lines[Corners[s2][1]]],4))
                m1 = m2 = n1 = n2 = 0
                seen1 = []
                seen2 = []
                for num1 in P1:
                    if num1 in seen1:
                        n1 = 1
                        Num1 = num1
                    else:
                        seen1.append(num1)
                for num2 in P2:
                    if num2 in seen2:
                        n2 = 1
                        Num2 = num2
                    else:
                        seen2.append(num2)
                if n1 == n2 == 1:
                    for k in xrange(4):
                        if P1[k] in P2:
                            m1 += 1
                        if P2[k] in P1:
                            m2 += 1
               
                if (m1 == m2 == 3)  and (Num1 != Num2):
                    for k in P1:
                        if (k != Num1) and (k != Num2):
                            O1 = k
                    for k in P2:
                        if (k != Num1) and (k != Num2):
                            O2 = k
                    O = [O1,O2]
                    O.sort()
                    if not O in Lines:
                        continue
                    else:
                        L0 = Length[Lines.index(O)]
                        L1 = Length[Corners[s1][0]]
                        L2 = Length[Corners[s1][1]]
                        L3 = Length[Corners[s2][0]]
                        L4 = Length[Corners[s2][1]]
                        L = np.array([L0, L1, L2, L3, L4])
                        if L.std() / L.mean() < 0.1:               
                            M = np.concatenate((P1,P2), axis = 0)
                            MarkerPts = []
                            for k in M:
                                if k in MarkerPts:
                                    continue
                                else:
                                    MarkerPts.append(k)

                            self.MarkerCoord = []
                            for x in xrange(4):
                                self.frameC = cv2.circle(self.frameC,tuple(ClusterCoord[MarkerPts[x]].astype(int)), 12, (0,255,0), -1)
                                self.MarkerCoord.append(ClusterCoord[MarkerPts[x]].astype(int))
                                Area[x] = cv2.contourArea(Clusters[MarkerPts[x]])
                                    
                            self.MarkerFound = 1
                            break
                   
                if self.MarkerFound:
                    break

    def marker_search(self):
        ret, self.frameC = self.cap.read()
        
        if self.MarkerFound:
             self.TrackMarker()   
        else:
             self.FindMarker()
            
        # Draw rectangle for marker reference
        cv2.rectangle(self.frameC, (self.W / 2, self.H / 2), (self.W / 2 + int(self.MeanLength), self.H / 2 + int(self.MeanLength)), (0,255,0), 1)
        
        # Reduce frame in order to show within widget
        RedframeC = cv2.resize(self.frameC, (self.W / 2, self.H / 2))
        self.RedframeC = cv2.cvtColor(RedframeC, cv2.COLOR_BGR2RGB)
