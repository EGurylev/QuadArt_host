from scipy import signal
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np
import math
import cv2

plt.ion()

#############################################################
def dist(p1, p2):
    return math.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))
#############################################################


#############################################################
def defineLine(p1, p2, A, D):
    #construct line for each pair of points: calculate angle
    #and length:
    p1 = p1.astype(float)
    p2 = p2.astype(float)

    #calulate angle of line:
    a = math.atan((p2[1] - p1[1]) / (p2[0] - p1[0]))
   
    #calculate length of this line
    d = dist(p1, p2)

    A.append(a)
    D.append(d)
    return 0
#############################################################

#############################################################
def isSide(AList, LengthList, AreaList):
    #test given tuple of lines to be a parallel sides of 
    #parallelogram. Lines must be parallel to each other,
    #equally spaced and have equal areas of their "points" 
    #(blob clusters).

    Tol = 0.1#tolerance band
    
    #are the triples parallel to each other?
    dA = abs(AList[0] - AList[1]) / ((abs(AList[0]) + abs(AList[1])) / 2)
  
    #Do the lines have equal areas of end "points"
    dArea = AreaList.std() / AreaList.mean()

    #are the triples have equal length?
    dL = abs(LengthList[0] - LengthList[1]) / ((abs(LengthList[0]) + abs(LengthList[1])) / 2)

    if (dA < Tol) and (dL < Tol) and (dArea < Tol):
        return 1
    else:
        return 0
#############################################################

#Initial settings
MinArea = 25
MaxArea = 35


#Load image
im = misc.imread('Tuple0.jpg')
W = im.shape[0]
L = im.shape[1]
im[im < 240] = 0

#Find separate blob clusters (line "points") as external contours
_, contours0, hierarchy = cv2.findContours( im.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

#Choose blob clusters only in specified area region
Clusters = []
for k in xrange(len(contours0)):
    if len(contours0[k]) > MinArea and len(contours0[k]) < MaxArea:
        Clusters.append(contours0[k])
Clusters = np.array(Clusters)

plt.imshow(im)
ClusterCoord = np.zeros([Clusters.shape[0], 2])
#Y = np.zeros(Clusters.shape[0])

#Find clusters' centres
for i in xrange(Clusters.shape[0]):
  ClusterCoord[i] = [Clusters[i][:,0][:,0].mean().round(), Clusters[i][:,0][:,1].mean().round()]
  plt.text(ClusterCoord[i,0],ClusterCoord[i,1],i)

ClusterCoord = ClusterCoord.astype(int)

Lines = []
Sides = []
Length = []#list of lengths of lines
A = []#list of angles for lines

#Cnstruct line objects
for p1 in xrange(ClusterCoord.shape[0]):
    for p2 in xrange(p1 + 1,ClusterCoord.shape[0]):
        if p1 == p2:
            continue
        else:
            defineLine(ClusterCoord[p1,:],ClusterCoord[p2,:], A, Length)
            Lines.append(sorted([p1, p2]))
            print p1, p2, A[-1], Length[-1]

#Test lines to be part of the parallelogram
for l1 in xrange(len(Lines)):
    for l2 in xrange(l1 + 1,len(Lines)):
        if l1 == l2:
            continue
        else:
            AList = [A[l1], A[l2]]
            LengthList = [Length[l1], Length[l2]]
            AreaList = np.array([len(Clusters[Lines[l1]][0]), len(Clusters[Lines[l1]][1]), len(Clusters[Lines[l2]][0]), len(Clusters[Lines[l2]][1])])
            if isSide(AList, LengthList, AreaList):
                Sides.append(sorted([l1, l2]))
                print Lines[Sides[-1][0]], Lines[Sides[-1][1]]

Sides = np.array(Sides)
flag = 0
for s1 in xrange(len(Sides)):
    for s2 in xrange(len(Sides)):
        P1 = Lines[Sides[s1][0]], Lines[Sides[s1][1]]
        P2 = Lines[Sides[s2][0]], Lines[Sides[s2][1]]
        if np.all(np.transpose(P1) == P2):
            print "Marker is found!"
            print P1
            flag = 1
            MarkerPts = np.reshape(P1, 4)
            plt.plot(ClusterCoord[MarkerPts][:,0], ClusterCoord[MarkerPts][:,1],'o')
            break
    if flag:
        break

