import csv
import matplotlib.pyplot as plt 
import numpy as np
import os
import ast
from math import sqrt

odom_xs = []
odom_ys = []
filename = os.path.join("2021-04-16-15-36-33_results", "2021-04-16-15-36-33.bag_odom.csv")
with open(filename) as csvfile:
    r = csv.reader(csvfile, delimiter=',', quotechar='|')
    for ix, row in enumerate(r):
        if ix > 225:
            print(row)
            sec, nsec, x, y = row
            odom_xs.append(float(x))
            odom_ys.append(float(y))

pose_xs = []
pose_ys = []
filename = os.path.join("2021-04-16-15-36-33_results", "2021-04-16-15-36-33.bag_pose.csv")
with open(filename) as csvfile:
    r = csv.reader(csvfile, delimiter=',', quotechar='|')
    for ix, row in enumerate(r):
        if ix != 0:
            print(row)
            sec, nsec, x, y = row
            pose_xs.append(float(x))
            pose_ys.append(float(y))



# plt.figure()
# plt.plot(odom_xs,odom_ys, color='k')
# plt.plot(pose_xs,pose_ys, color='r')
# plt.xlabel("x position")
# plt.ylabel("y position")
# plt.title("Localization vs. Ground Truth")
# plt.legend(["Ground Truth", "Localization"])
# plt.show()

##############################
# POSE ARRAY STD OVER TIME 
##############################

avgs = []
stds = []

filename = os.path.join("2021-04-16-15-36-33_results", "2021-04-16-15-36-33.bag_avg.csv")
with open(filename) as csvfile:
    r = csv.reader(csvfile, delimiter=',', quotechar='|')
    for ix, row in enumerate(r):
        if ix != 0: # skip the header
            sec, nsec, x_avg, y_avg = row
            avgs.append((float(x_avg), float(y_avg)))

filename = os.path.join("2021-04-16-15-36-33_results", "2021-04-16-15-36-33.bag_posearray.csv")
with open(filename) as csvfile:
    r = csv.reader(csvfile, delimiter=';', quotechar='|')
    for ix, row in enumerate(r):
        if ix != 0 and ix < len(avgs):
            sec, nsec, poses = row
            poses = np.array(ast.literal_eval(poses))
            dists = []
            for pose in poses: 
                x,y=pose
                dists.append(sqrt( (float(x)-avgs[ix][0])**2 + (float(y)-avgs[ix][1])**2  ))
            stds.append(np.std(np.array(dists)))

plt.figure()
plt.plot(stds)
plt.xlabel("time")
plt.ylabel("std")
plt.title("Standard Deviation Convergence of Particles")
plt.legend(["x", "y"])
plt.show()


