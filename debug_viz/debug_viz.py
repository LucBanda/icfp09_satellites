#!/usr/bin/env python3
import numpy as np
import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys


if (len(sys.argv) == 1):
    path = "../vm/build/debug/4001/"
    index = 0
else:
    path = sys.argv[1]
    index = sys.argv[2]

datas = np.genfromtxt(path+"/" + str(index) + ".txt", delimiter = '\t')


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

max_score = -1000

for chromosome in datas:
    if chromosome[0] > max_score:
        max_score = chromosome[0]

i = 0
j = 0
for chromosome in datas:

    if chromosome[0] > 50:
        c = 'y'
        i += 1
        if i % 10 == 0:
            xs = chromosome[2]
            ys = chromosome[3]
            zs = chromosome[1]
            ax.scatter(xs, ys, zs, c=c, marker='.')

    if chromosome[0] > max_score - max_score / 100.:
        c = 'r'
        xs = chromosome[2]
        ys = chromosome[3]
        zs = chromosome[1]
        ax.scatter(xs, ys, zs, c=c, marker='.')
    elif chromosome[0] > max_score - max_score * 5. / 100.:
        c = 'b'
        j += 1
        if j % 5 == 0:
            xs = chromosome[2]
            ys = chromosome[3]
            zs = chromosome[1]
            ax.scatter(xs, ys, zs, c=c, marker='.')


ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('time Label')
plt.show()