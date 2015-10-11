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
def defineLine(p1, p2, L):
    #construct line for each pair of points: calculate angle
    #and length:
    p1 = p1.astype(float)
    p2 = p2.astype(float)
   
    #calculate length of this line
    l = dist(p1, p2)

    L.append(l)
    return 0
#############################################################

#############################################################
def isSide(LengthList, AreaList, CentersList):
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

#############################################################
def FindMarker(frame):
   #Find separate blob clusters (line "points") as external contours
   _, contours0, hierarchy = cv2.findContours( frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

   #Choose blob clusters only in specified area region
   Clusters = []
   for k in xrange(len(contours0)):
       if len(contours0[k]) > MinArea and len(contours0[k]) < MaxArea:
           Clusters.append(contours0[k])
   Clusters = np.array(Clusters)

   plt.imshow(frame)
   ClusterCoord = np.zeros([Clusters.shape[0], 2])

   #Find clusters' centres
   for i in xrange(Clusters.shape[0]):
     ClusterCoord[i] = [Clusters[i][:,0][:,0].mean().round(), Clusters[i][:,0][:,1].mean().round()]
     plt.text(ClusterCoord[i,0],ClusterCoord[i,1],i)

   ClusterCoord = ClusterCoord.astype(int)

   Lines = []
   Corners = []
   Length = []#list of lengths of lines

   #Cnstruct line objects
   for p1 in xrange(ClusterCoord.shape[0]):
       for p2 in xrange(p1 + 1,ClusterCoord.shape[0]):
           if p1 == p2:
               continue
           else:
               defineLine(ClusterCoord[p1,:],ClusterCoord[p2,:], Length)
               Lines.append(sorted([p1, p2]))
               print p1, p2, Length[-1]

   #Test lines to be part of the parallelogram
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
               if isSide(LengthList, AreaList, CentersList):
                   Corners.append(sorted([l1, l2]))
                   print Lines[Corners[-1][0]], Lines[Corners[-1][1]]

   Corners = np.array(Corners)
   flag = 0
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
               L0 = Length[Lines.index(O)]
               L1 = Length[Corners[s1][0]]
               L2 = Length[Corners[s1][1]]
               L3 = Length[Corners[s2][0]]
               L4 = Length[Corners[s2][1]]
               L = np.array([L0, L1, L2, L3, L4])
               if L.std() / L.mean() < 0.1:
                   print "Marker is found!"
                   print P1,P2
                   print Lines[Corners[s1][0]], Lines[Corners[s1][1]]
                   print Lines[Corners[s2][0]], Lines[Corners[s2][1]]
               
                   M = np.concatenate((P1,P2), axis = 0)
                   MarkerPts = []
                   for k in M:
                       if k in MarkerPts:
                           continue
                       else:
                           MarkerPts.append(k)
                   plt.plot(ClusterCoord[MarkerPts][:,0], ClusterCoord[MarkerPts][:,1],'o')
                   flag = 1
                   break
               
       if flag:
           break
   return (Clusters,Lines,Corners,Length)
#############################################################

#Start
#Initial settings
MinArea = 28
MaxArea = 36
#plt.ion()
frame = misc.imread('Tuple2.jpg')
frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#_, frame = cv2.threshold(frame,150,255,cv2.THRESH_BINARY)
th2 = cv2.adaptiveThreshold(frame,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,13,-3)
Clusters,Lines,Corners,Length = FindMarker(th2)

#cv windows
flagCV = 0
if flagCV:
    cap = cv2.VideoCapture(0)
    cv2.namedWindow('Trackbars')
    cv2.createTrackbar('Thresh','Trackbars',0,255,nothing)
    cv2.createTrackbar('MinArea','Trackbars',0,255,nothing)
    cv2.createTrackbar('MaxArea','Trackbars',0,255,nothing)

    while(True):
        ret, frameC = cap.read()
        W = frameC.shape[0]
        L = frameC.shape[1]
        frame = cv2.cvtColor(frameC, cv2.COLOR_BGR2GRAY)
        th = cv2.getTrackbarPos('Thresh','Trackbars')
        MinArea = cv2.getTrackbarPos('MinArea','Trackbars')
        MaxArea = cv2.getTrackbarPos('MaxArea','Trackbars')
        _, frame = cv2.threshold(frame,th,255,cv2.THRESH_BINARY)
        _, contours0, hierarchy = cv2.findContours( frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        #Choose blob clusters only in specified area region
        Clusters = []
        for k in xrange(len(contours0)):
            if len(contours0[k]) > MinArea and len(contours0[k]) < MaxArea:
                Clusters.append(contours0[k])
    
        debugInfo = []
        debugInfo.append(str(len(Clusters)))
        debugInfo.append(str(th))
    
        DebugImg = np.zeros((150,80,3), dtype=np.int8)
        for t in xrange(len(debugInfo)): 
            cv2.putText(DebugImg,debugInfo[t],(25,25+15*t), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255,255,255),1,cv2.LINE_AA)
        cv2.fillPoly(frameC, pts = Clusters, color=(0,0,255))
        cv2.imshow('Trackbars',DebugImg)
        cv2.imshow('frame',frameC)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
