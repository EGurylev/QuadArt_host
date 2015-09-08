from scipy import signal
from scipy import misc
import matplotlib.pyplot as plt
import numpy as np

plt.ion()

im = misc.imread('/home/evgeniy/Documents/Py/Quadrotor/Image_9.jpg')
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
X = np.zeros(Clusters.shape[0])
Y = np.zeros(Clusters.shape[0])

#Find clusters' centres
for i in xrange(Clusters.shape[0]):
  X[i] = PointCoord[Clusters[i]][:,0].mean().round()
  Y[i] = PointCoord[Clusters[i]][:,1].mean().round()
plt.imshow(im)
plt.plot(Y,X,'o')


