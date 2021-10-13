import csv
import matplotlib.pyplot as plt 
import numpy as np
from scipy.ndimage.interpolation import zoom
import os
import ast
from math import sqrt

# plt.figure()

MAP_RES = 0.0504

start_indices = {
    '2021-04-28-02-44-28': 774, 
    '2021-04-28-02-46-22': 1666, 
    '2021-04-28-02-48-13': 824, 
    '2021-04-28-03-08-31': 713, 
    '2021-04-28-03-10-28': 837, 
    '2021-04-28-03-12-49': 598, 
    '2021-04-28-03-18-34': 649, 
}

for d, ind in start_indices.items():
    t = []
    odom_xs = []
    odom_ys = []
    # filename = os.path.join("2021-04-28-02-44-28_results", "2021-04-28-02-44-28.bag_odom.csv")
    filename = os.path.join(d+'_results', d+'.bag_odom.csv')
    with open(filename) as csvfile:
        r = csv.reader(csvfile, delimiter=',', quotechar='|')
        for ix, row in enumerate(r):
            if ix > ind:
                sec, nsec, x, y = row
                if abs(float(x))>0.1 and abs(float(y))>0.1:
                    t.append(float(sec)+float(nsec)*10**-9)
                    odom_xs.append(float(x))
                    odom_ys.append(float(y))

    traj_xs = []
    traj_ys = []
    # filename = os.path.join("2021-04-28-02-44-28_results", "2021-04-28-02-44-28.bag_traj.csv")
    filename = os.path.join(d+'_results', d+'.bag_traj.csv')
    with open(filename) as csvfile:
        r = csv.reader(csvfile, delimiter=',', quotechar='|')
        for ix, row in enumerate(r):
            if ix != 0:
                x, y = row
                traj_xs.append(float(x))
                traj_ys.append(float(y))

    interp_traj_xs = zoom(traj_xs, zoom=len(odom_xs)/float(len(traj_xs)), order=1)
    interp_traj_ys = zoom(traj_ys, zoom=len(odom_ys)/float(len(traj_ys)), order=1)

    err_over_time = []
    for i in range(len(interp_traj_xs)):
        err_over_time.append( sqrt( (interp_traj_xs[i] - odom_xs[i])**2 + (interp_traj_ys[i] - odom_ys[i])**2 ) * MAP_RES * 100)

    print(str(sum(err_over_time)/len(err_over_time))+',')
    
    # plt.figure()
    fig, (ax1, ax2) = plt.subplots(2)
    ax1.plot(odom_xs,odom_ys, color='k')
    ax1.plot(interp_traj_xs, interp_traj_ys, color='r')
    ax1.grid()
    ax1.set_xlabel("px")
    ax1.set_ylabel("py")
    ax1.set_title("Planned Trajectory vs. Odometry")
    ax1.legend(["Odometry", "Planned"])

    ax2.plot(np.array(t)-t[0],err_over_time)
    ax2.set_title("Error vs. Time")
    ax2.set_ylabel("Euclidean Error (cm)")
    ax2.set_xlabel("Time (sec)")
    plt.tight_layout()
    plt.show()

print(np.mean([
0.197686162579,
1.03747610388,
1.06215046798,
1.51960073586,
1.09627296922,
2.51548455525,
0.252440710298,
]))