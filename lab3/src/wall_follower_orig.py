#!/usr/bin/env python2
import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from math import pi, sqrt
#from sklearn import linear_model, datasets

#TODO(gcfchen):
# make it nearsighted
# pi/2 to front-ish?
# front distances
# tune kp kd


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower_tesse/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower_tesse/drive_topic")
    SIDE = rospy.get_param("wall_follower_tesse/side")
    VELOCITY = rospy.get_param("wall_follower_tesse/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower_tesse/desired_distance")

    front_angles, left_angles, right_angles = [], [], []
    front_distances, left_distances, right_distances = [], [], []
    prev_err = 0

    err_history = []
    err_history_size = 25

    min_dist_history = []
    min_dist_history_size = 10 # ADJUST THIS FOR MOVING AVERAGE LOW PASS FILTER ON VERTICAL DISTANCE (FIXES NOISE)

    total_history = []

    def __init__(self):
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=50)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self._line_marker = Marker()
        self._line_marker.header.frame_id = "/base_link_gt"
        self._line_marker.type = self._line_marker.LINE_STRIP
        self._line_marker.action = self._line_marker.ADD
        t = rospy.Duration()
        self._line_marker.lifetime = t
        self._line_marker.scale.x = 0.4
        self._line_marker.scale.y = 0.4
        self._line_marker.scale.z = 0.4
        self._line_marker.color.a = 1.0
        self._line_marker.color.r = 1.0

    def callback(self, data):
        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        scan_ranges = np.array(data.ranges)
        size = len(scan_ranges)

        angles = np.array([angle_min + i*angle_increment for i in range(0,size)])
        cosine = np.cos(angles)
        sine = np.sin(angles)

        x_comp = np.multiply(scan_ranges, cosine)
        y_comp = np.multiply(scan_ranges, sine)

        if self.SIDE == -1: #right side
            x = x_comp[size//6:size//2] #only want data on that side, don't need data behind us
            y = y_comp[size//6:size//2]
            dists = scan_ranges[size//6:size//2]

        else:#left side
            x = x_comp[size//2:5*size//6]
            y = y_comp[size//2:5*size//6]
            dists = scan_ranges[size//2:5*size//6]

        std = np.std(dists)
        avg = np.mean(dists)
        i = ((dists>(avg-2*std)) & (dists<(avg+2*std)) )#get indices
        x = x[i] #only want relevant data, no outliers
        y = y[i]

        #coefs = self.least_squares(x,y) ransac doesn't work yet leave commented
        m,b = self.least_squares(x,y)
        # rospy.loginfo("%s,%s", 'COEFS OUT: ',coefs)
#here
        self._line_marker.points=[]
        for i in range(len(x)):
            p = Point()
            p.x = x[i]
            p.y = m*x[i]+b
            p.z = 0
            self._line_marker.points.append(p)
        self.marker_publisher.publish(self._line_marker)
        
        kp = 2.0
        kd = 5.0
        ki = 1.0
        
        theta = np.arctan(m)
        min_dist = abs(b)/sqrt(m**2+1.0)
        
        self.min_dist_history.append(min_dist)
        if len(self.min_dist_history) > self.min_dist_history_size:
            self.min_dist_history.pop(0)
        min_dist_avg = np.mean(self.min_dist_history)
        self.total_history.append(min_dist_avg)
        
        error_dist = -self.SIDE*(abs(min_dist_avg) - self.DESIRED_DISTANCE) #for kp
        error_velocity = -self.VELOCITY*np.sin(theta) #for kd
        
        # for ki
        self.err_history.append(error_dist)
        if len(self.err_history) > self.err_history_size:
            self.err_history.pop(0)
        err_int = -np.sum(self.err_history) * (1/25.0)
        
        steering_angle = (kp*error_dist + kd*error_velocity + ki*err_int)
        # rospy.loginfo(data.ranges)
        rospy.loginfo("%s, %s, %s, %s, %s", self.DESIRED_DISTANCE, min_dist, round(kp*error_dist,2), round(kd*error_velocity, 2), round(steering_angle,2))
        
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = self.VELOCITY
        self.pub.publish(msg)
#here
    def least_squares(self, x, y):
        # original least squares code
        A = np.vstack([x, np.ones(len(x))]).T
        m,b = np.linalg.lstsq(A,y,rcond = None)[0]
        return m,b

        #new ransac code doesn't work yet don't uncomment 
        # ransac = linear_model.RANSACRegressor()
        # ransac.fit(x,y)
        # inlier_mask = ransac.inlier_mask_
        # outlier_mask = np.logical_not(inlier_mask)
        # coefs = ransac.estimator_.coef_
        # return coefs


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()