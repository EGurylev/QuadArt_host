from scipy import signal
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np
import math

plt.ion()

def isPointOnGrid(p1, p2, p3):
    p1 = p1.astype(float)
    p2 = p2.astype(float)
    p3 = p3.astype(float)
    Tol = 0.1#tolerance band
    #are these points on the same line?
    #calulate a and b from p1 and p2
    a = (p2[1] - p1[1]) / (p2[0] - p1[0])
    b = p1[1] - a * p1[0]
    y3calc = a * p3[0] + b
    #are these points equally spaced?
    #calculate distance between them
    d12 = math.sqrt(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2))
    d13 = math.sqrt(pow(p1[0] - p3[0], 2) + pow(p1[1] - p3[1], 2))
    d23 = math.sqrt(pow(p2[0] - p3[0], 2) + pow(p2[1] - p3[1], 2))
    #Confitions
    CondL = abs(y3calc - p3[1]) / p3[1] < Tol
    CondD1 = (abs(d12 - d23) / d12 < Tol) and (abs(d13 - 2 * d23) / d13 < Tol)
    CondD2 = (abs(d12 - d13) / d12 < Tol) and (abs(d23 - 2 * d13) / d23 < Tol)

    if CondL and (CondD1 or CondD2):
       return 1
    else:
       return 0


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



#Test these points to be on grid

for p1 in xrange(ClusterCoord.shape[0]):
    for p2 in xrange(ClusterCoord.shape[0]):
        for p3 in xrange(ClusterCoord.shape[0]):
            if p1 == p2 or p1 == p3 or p2 == p3:
                continue
            else:
                if isPointOnGrid(ClusterCoord[p1,:],ClusterCoord[p2,:],ClusterCoord[p3,:]):
                    print p1, p2, p3
                    plt.plot(ClusterCoord[p1,1],ClusterCoord[p1,0],'o')
		    plt.plot(ClusterCoord[p2,1],ClusterCoord[p2,0],'o')
		    plt.plot(ClusterCoord[p3,1],ClusterCoord[p3,0],'o')
		    

#1,3,4,6,7,8,9,10,11
