from scipy import signal
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2

def nothing(x):
    pass

#############################################################
def dist(p1, p2):
    return math.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))
#############################################################

#############################################################
def MeanShift(frame,MarkerCoord):
    MarkerCoordUpd = MarkerCoord
    #Search with fixed number of steps
    NumOfMeanShiftSteps = 3
    WindowSize = 50#todo: should depend on scale
    #plt.imshow(frame)
    #plt.plot(MarkerCoord[0],MarkerCoord[1],'o')
    for i in xrange(NumOfMeanShiftSteps):
        XWindowLeft = MarkerCoordUpd[1] - WindowSize / 2
        XWindowRigth = MarkerCoordUpd[1] + WindowSize / 2
        YWindowDown = MarkerCoordUpd[0] - WindowSize / 2
        YWindowUp = MarkerCoordUpd[0] + WindowSize / 2
        frameW = frame[XWindowLeft:XWindowRigth,YWindowDown:YWindowUp]
        _, contours, hierarchy = cv2.findContours( frameW.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        Area = []
        for cont in contours:
            Area.append(cv2.contourArea(cont))
        Area = np.array(Area)
        if Area.size:
            idx = Area.argmax()
            C = contours[idx]
            N = C.shape[0]
            C = C.squeeze()
            C = C.reshape(N * 2, 1)
            Y = np.array(C[1::2])
            X = np.array(C[0::2])           
            MarkerCoordUpd += np.array([X.mean().round(), Y.mean().round()]) - WindowSize / 2
            #plt.plot(MarkerCoordUpd[0],MarkerCoordUpd[1],'o')
            
        else:
            return (np.array([0, 0]), 0)
    
    return (MarkerCoordUpd, Area[idx])
#############################################################


#############################################################
def isLine(p1, p2):
    #construct line with length in specific range
    p1 = p1.astype(float)
    p2 = p2.astype(float)
   
    #calculate length of this line
    l = dist(p1, p2)
    if l < MaxLength and l > MinLength:
        return l
    else:
        return 0
#############################################################

#############################################################
def isCorner(LengthList, AreaList, CentersList):
    #test given tuple of lines to be perpendicular to each 
    #other and have equal lengths and equal areas of their
    # "points" (blob clusters).

    Tol = 0.1#tolerance band

    #are the adjacent lines perpendicular to each other?
    V1 = [CentersList[1][0]-CentersList[0][0], CentersList[1][1]-CentersList[0][1]]
    V2 = [CentersList[3][0]-CentersList[2][0], CentersList[3][1]-CentersList[2][1]]
    temp = np.dot(V1,V2) / (LengthList[0] * LengthList[1])
    if temp > 1:
        temp = 1
    elif temp < -1:
        temp = -1
    alpha = math.acos(temp)
    dA  = abs(math.pi/2 - abs(alpha))
    
    #Do the lines have equal areas of end "points"
    dArea = AreaList.std() / AreaList.mean()

    #are the triples have equal length?
    dL = abs(LengthList[0] - LengthList[1]) / ((abs(LengthList[0]) + abs(LengthList[1])) / 2)

    if (dL < Tol) and (dArea < Tol) and (dA < Tol):
        return 1
    else:
        return 0
#############################################################


#Start
#Initial settings
MinArea = 50
MaxArea = 80
MinLength = 70
MaxLength = 90
LengthRef = (MinLength + MaxLength) / 2

#cv windows
flagCV = 1
if flagCV:
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
    cv2.namedWindow('Debug')
    MarkerFound = 0

    while(True):
        nt1 = cv2.getTickCount()
        ret, frameC = cap.read()
        #picname = ['pic/Pic', str(nf), '.jpg']
        #picname = ''.join(picname)
        #frameC = cv2.imread(picname)
        W = frameC.shape[0]
        H = frameC.shape[1]
        frame = cv2.cvtColor(frameC, cv2.COLOR_BGR2GRAY)
        frame = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,13,-3)
        if MarkerFound:
            for m in xrange(4):
                MarkerCoord[m],Area[m] = MeanShift(frame,MarkerCoord[m])
                if not MarkerCoord[m].any():
                    MarkerFound = 0
                    break
                else:
                    frameC = cv2.circle(frameC,tuple(MarkerCoord[m].astype(int)), 5, (0,255,255), -1)
        else:
            _, contours0, hierarchy = cv2.findContours( frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
            #Choose blob clusters only in specified area region
            Clusters = []
            for k in xrange(len(contours0)):
                if cv2.contourArea(contours0[k]) > MinArea and cv2.contourArea(contours0[k]) < MaxArea:
                    Clusters.append(contours0[k])
                
            Clusters = np.array(Clusters)
            ClusterCoord = np.zeros([Clusters.shape[0], 2])
            cv2.fillPoly(frameC, pts = Clusters, color=(0,0,255))
        
            #Find clusters' centres
            for i in xrange(Clusters.shape[0]):
                ClusterCoord[i] = [Clusters[i][:,0][:,0].mean().round(), Clusters[i][:,0][:,1].mean().round()]

            ClusterCoord = ClusterCoord.astype(int)

            Lines = []
            Length = []
            Corners = []

            #Construct line objects
            for p1 in xrange(ClusterCoord.shape[0]):
                for p2 in xrange(p1 + 1,ClusterCoord.shape[0]):
                    if p1 == p2:
                       continue
                    else:
                       L = isLine(ClusterCoord[p1,:],ClusterCoord[p2,:])
                       if L:
                           Length.append(L)
                           Lines.append(sorted([p1, p2]))
                           frameC = cv2.line(frameC,tuple(ClusterCoord[p1,:]),\
                               tuple(ClusterCoord[p2,:]),(255,0,0),1)

            #Test lines to be perpendicular and equal
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

                                MarkerCoord = []
                                for x in xrange(4):
                                    frameC = cv2.circle(frameC,tuple(ClusterCoord[MarkerPts[x]].astype(int)), 12, (0,255,0), -1)
                                    MarkerCoord.append(ClusterCoord[MarkerPts[x]].astype(int))
                                    Area[x] = cv2.contourArea(Clusters[MarkerPts[x]])
                                
                                MarkerFound = 1
                            
                                MinArea = np.round(Area.min() - 0.1 * Area.mean())
                                MaxArea = np.round(Area.max() + 0.1 * Area.mean())
                                MinLength = np.round(L.min() - 0.1 * L.mean())
                                MaxLength = np.round(L.max() + 0.1 * L.mean())
                                break
               
                    if MarkerFound:
                        break
        
        
        #plt.imshow(frameF)
        tf = cv2.getTickFrequency()
        nt2 = cv2.getTickCount()
        fps = cap.get(cv2.CAP_PROP_FPS)
        debugInfo = []
        debugInfo.append(str(len(Clusters)))
        debugInfo.append(str(tf * 1 / (nt2 - nt1)))
        debugInfo.append(str(1000 * ((nt2 - nt1) / tf)))
        debugInfo.append(str(MinArea))
        debugInfo.append(str(MaxArea))
        debugInfo.append(str(MinLength))
        debugInfo.append(str(MaxLength))

        #Draw rectangle for marker reference
        cv2.rectangle(frameC, (H / 2, W / 2), (H / 2 + LengthRef, W / 2 + LengthRef), (0,255,0), 1)
    
        DebugImg = np.zeros((150,120,3), dtype=np.int8)
        for t in xrange(len(debugInfo)): 
            cv2.putText(DebugImg,debugInfo[t],(25,25+15*t), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255,255,255),1,cv2.LINE_AA)
        cv2.imshow('Debug',DebugImg)
        cv2.imshow('frame',frameC)
        #cv2.imshow('Clusters',frame)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
