from scipy import signal
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np
import math

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
def isGrid(t1, t2, t3, AList):
    #test given three triples to be a grid. Triples must be
    #parallel to each other and equally spaced
    
    #are the triples parallel to each other?
    Tol = 0.15#tolerance band
    dA12 = abs(AList[0] - AList[1]) / ((abs(AList[0]) + abs(AList[1])) / 2) < Tol
    dA13 = abs(AList[0] - AList[2]) / ((abs(AList[0]) + abs(AList[2])) / 2) < Tol
    dA23 = abs(AList[1] - AList[2]) / ((abs(AList[1]) + abs(AList[2])) / 2) < Tol
    if dA12 and dA13 and dA23:
        return 1
    else:
        return 0
#############################################################


im = misc.imread('Image_9.jpg')
W = im.shape[0]
L = im.shape[1]
NumOfPoints = np.count_nonzero(im == 255)
flag = np.zeros(NumOfPoints)
R = 25
Clusters = []
PointCoord = []
seed = 0;
seedSet = 1;
clustering = 1


for i in xrange(W):
  for j in xrange(L):
    if im[i][j] == 255:
      PointCoord.append([i, j])
while clustering:
  Cluster = []
  for n in range(NumOfPoints):
    if not(flag[n]) and not(seedSet):
      seed = n
      seedSet = 1
    if ((abs(PointCoord[seed][0] - PointCoord[n][0]) < R) and (abs(PointCoord[seed][1] - PointCoord[n][1]) < R) and seedSet):
      Cluster.append(n)
      flag[n] = 1
  seedSet = 0
  Clusters.append(Cluster)
  if flag[NumOfPoints-1]:
    clustering = 0

Clusters = np.array(Clusters)
PointCoord = np.array(PointCoord)

plt.imshow(im)
ClusterCoord = np.zeros([Clusters.shape[0], 2])
Y = np.zeros(Clusters.shape[0])

#Find clusters' centres
for i in xrange(Clusters.shape[0]):
  ClusterCoord[i] = [PointCoord[Clusters[i]][:,0].mean().round(), PointCoord[Clusters[i]][:,1].mean().round()]
  plt.text(ClusterCoord[i,1],ClusterCoord[i,0],i)

ClusterCoord = ClusterCoord.astype(int)
plt.imshow(im)

Triples = []
Grids = []
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
                    print p1, p2, p3, A[-1]

print

#Test 3 triples to be the grid:
for t1 in xrange(len(Triples)):
    for t2 in xrange(len(Triples)):
        for t3 in xrange(len(Triples)):
            Cond1 = ((t1 == t2) or (t1 == t3) or (t2 == t3))
            Cond2 = ((Triples[t1][0] == Triples[t2][0]) or (Triples[t1][0] == Triples[t3][0]) or (Triples[t2][0] == Triples[t3][0]))
            Cond3 = sorted([Triples[t1], Triples[t2], Triples[t3]]) in Grids
            if Cond1 or Cond2 or Cond3:
                continue
            else:
                AList = [A[t1], A[t2], A[t3]]
                if isGrid(t1, t2, t3, AList):
                    Grids.append(sorted([Triples[t1], Triples[t2], Triples[t3]]))
		    print Triples[t1]
                    print Triples[t2]
                    print Triples[t3]
                    print "..."

