#!/usr/bin/env python2
import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=50)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)

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
        i = ((dists>(avg-std)) & (dists<(avg+std)))#get indices
        x = x[i] #only want relevant data, no outliers
        y = y[i]

        m,b = self.least_squares(x,y)

        kp = 5
        kd = 8
        theta = np.arctan(m)
        min_dist = b*np.cos(theta)

        error_dist = self.SIDE*(abs(min_dist) - self.DESIRED_DISTANCE) #for kp
        error_velocity = self.VELOCITY*np.sin(theta) #for kd
        steering_angle = (kp*error_dist + kd*error_velocity)

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = steering_angle
        msg.drive.speed = self.VELOCITY
        self.pub.publish(msg)

    def least_squares(self, x, y):
        A = np.vstack([x, np.ones(len(x))]).T
        m,b = np.linalg.lstsq(A,y,rcond = None)[0]
        return m,b

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
