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
def isTriple(p1, p2, p3, A):
    #test given three points to be triple. Triple is an object
    #which has three equally spaced points lying on the one line
    p1 = p1.astype(float)
    p2 = p2.astype(float)
    p3 = p3.astype(float)
    Tol = 0.15#tolerance band

    #are these points on the same line?
    #calulate a and b

    #some exceptions: if two points have the same X coord
    #in order to avoid division by zero try another pair:
    #if all 3 points have the same X then it's a straight line
    if (p1[0] == p2[0]) and (p1[0] == p3[0]):
        CondL = 1#straight line
        a = 0
    elif p1[0] == p2[0]:
        a = (p3[1] - p1[1]) / (p3[0] - p1[0])
        b = p1[1] - a * p1[0]
        ycalc = a * p2[0] + b
        CondL = abs(ycalc - p2[1]) / p2[1] < Tol
    else:
        a = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = p1[1] - a * p1[0]
        ycalc = a * p3[0] + b
        CondL = abs(ycalc - p3[1]) / p3[1] < Tol
    
    #are these points equally spaced?
    #calculate distance between them
    d12 = dist(p1, p2)
    d13 = dist(p1, p3)
    d23 = dist(p2, p3)
    #Distance test
    CondD = (abs(d12 - d23) / ((d12 + d23) / 2) < Tol) and (abs(d13 - 2 * d23) / ((d13 + 2 * d23) / 2) < Tol)

    if CondL and CondD:
       A.append(a)
       return 1
    else:
       return 0
#############################################################

#############################################################
def isGrid(t1, t2, t3, AList, DistanceList):
    #test given three triples to be a grid. Triples must be
    #parallel to each other and equally spaced
    
    #are the triples parallel to each other?
    Tol = 0.15#tolerance band
    dA12 = abs(AList[0] - AList[1]) / ((abs(AList[0]) + abs(AList[1])) / 2)
    dA13 = abs(AList[0] - AList[2]) / ((abs(AList[0]) + abs(AList[2])) / 2)
    dA23 = abs(AList[1] - AList[2]) / ((abs(AList[1]) + abs(AList[2])) / 2)

    #are the triples have equal length?
    dD12 = abs(DistanceList[0] - DistanceList[1]) / ((abs(DistanceList[0]) + abs(DistanceList[1])) / 2)
    dD13 = abs(DistanceList[0] - DistanceList[2]) / ((abs(DistanceList[0]) + abs(DistanceList[2])) / 2)
    dD23 = abs(DistanceList[1] - DistanceList[2]) / ((abs(DistanceList[1]) + abs(DistanceList[2])) / 2)
    if ((dA12 + dA13 + dA23) / 3 < Tol) and ((dD12 + dD13 + dD23) / 3 < Tol):
        return 1
    else:
        return 0
#############################################################


im = misc.imread('Image5.jpg')
W = im.shape[0]
L = im.shape[1]
im[im < 240] = 0
#NumOfPoints = np.count_nonzero(im == 255)
Clusters = []
#PointCoord = []


_, contours0, hierarchy = cv2.findContours( im.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

Clusters = []
for k in xrange(len(contours0)):
    if len(contours0[k]) > 15 and len(contours0[k]) < 45:
        Clusters.append(contours0[k])

Clusters = np.array(Clusters)
#PointCoord = np.array(PointCoord)

plt.imshow(im)
ClusterCoord = np.zeros([Clusters.shape[0], 2])
Y = np.zeros(Clusters.shape[0])

#Find clusters' centres
for i in xrange(Clusters.shape[0]):
  ClusterCoord[i] = [Clusters[i][:,0][:,0].mean().round(), Clusters[i][:,0][:,1].mean().round()]
  plt.text(ClusterCoord[i,0],ClusterCoord[i,1],i)

ClusterCoord = ClusterCoord.astype(int)
plt.imshow(im)

Triples = []
Grids = []
Distance = []
A = []#list of line coefficients for triples (Ax + b)

#Test 3 points to be the triple
for p1 in xrange(ClusterCoord.shape[0]):
    for p2 in xrange(ClusterCoord.shape[0]):
        for p3 in xrange(ClusterCoord.shape[0]):
            Cond1 = ((p1 == p2) or (p1 == p3) or (p2 == p3))
            Cond2 = sorted([p1, p2, p3]) in Triples
            if Cond1 or Cond2:
                continue
            else:
                if isTriple(ClusterCoord[p1,:],ClusterCoord[p2,:],ClusterCoord[p3,:], A):
                    Triples.append(sorted([p1, p2, p3]))
		    Distance.append(dist(ClusterCoord[p1,:], ClusterCoord[p3,:]))
                    print p1, p2, p3, A[-1], Distance[-1]


#Test 3 triples to be parallel to each other:
for t1 in xrange(len(Triples)):
    for t2 in xrange(len(Triples)):
        for t3 in xrange(len(Triples)):
            Cond1 = ((t1 == t2) or (t1 == t3) or (t2 == t3))
            Cond2 = ((Triples[t1][0] == Triples[t2][0]) or (Triples[t1][0] == Triples[t3][0]) or (Triples[t2][0] == Triples[t3][0]))
            Cond3 = ((Triples[t1][1] == Triples[t2][1]) or (Triples[t1][1] == Triples[t3][1]) or (Triples[t2][1] == Triples[t3][1]))
            Cond4 = ((Triples[t1][2] == Triples[t2][2]) or (Triples[t1][2] == Triples[t3][2]) or (Triples[t2][2] == Triples[t3][2]))
            Cond5 = sorted([Triples[t1], Triples[t2], Triples[t3]]) in Grids
            if Cond1 or Cond2 or Cond3 or Cond4 or Cond5:
                continue
            else:
                AList = [A[t1], A[t2], A[t3]]
                DistanceList = [Distance[t1], Distance[t2], Distance[t3]]
                if isGrid(t1, t2, t3, AList, DistanceList):
                    Grids.append(sorted([Triples[t1], Triples[t2], Triples[t3]]))
		    #print Triples[t1]
                    #print Triples[t2]
                    #print Triples[t3]
                    #print "..."

#Test triples to form the grid:
Grids = np.array(Grids)
for g1 in xrange(Grids.shape[0]):
    for g2 in xrange(Grids.shape[0]):
        if np.all(np.transpose(Grids[g1]) == Grids[g2]):
            i = g1
            break

print Grids[i]
print "Grid is found!"
GridPts = np.reshape(Grids[i], 9)
plt.plot(ClusterCoord[GridPts][:,0], ClusterCoord[GridPts][:,1],'o')
