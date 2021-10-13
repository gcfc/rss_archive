#!/usr/bin/env python
import rospy
from visual_servoing_tesse.msg import cone_location, parking_error
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from math import *

class ParkingController():

    DRIVE_TOPIC = "/tesse/drive"
    CONE_LOCATION = "/relative_cone"
    PARKING_ERROR_TOPIC = "/parking_error"
    def __init__(self):
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size = 5)
        self.sub = rospy.Subscriber(self.CONE_LOCATION, cone_location, self.relative_cone_callback)
        self.error_pub = rospy.Publisher(self.PARKING_ERROR_TOPIC, parking_error, queue_size = 5)

    def relative_cone_callback(self,msg):
        x = msg.x_pos
        y = msg.y_pos
        L = 3.0

        # lookahead distance
        d = np.sqrt(x**(2)+y**(2))

        rospy.loginfo('x: %s \t y: %s \t d: %s', round(x,3), round(y,3), round(d,3))

        # desired_distance = d - 0.45
        #desired_distance = d - L/2 - 0.6 - 1.4
        desired_distance = 1
        theta = - atan2(y,x)
        steering_angle = atan(2*L*sin(theta)/desired_distance)

        # choosing velocity
        v_max = 3
        Kp = 1
        Kp2 = 0.5
        parked = False

        if desired_distance < 0.0: #and abs(steering_angle) < 0.05: #0.25:
            parked = True
            velocity = 0
        else:
            velocity = max(min(v_max, Kp * desired_distance) - Kp2 * abs(steering_angle),1)

        if parked: velocity = 0

        rospy.loginfo("D.D: %s \t S.A.: %s \t V: %s", round(desired_distance,3), round(steering_angle,3), round(velocity,3))

        Kp3 = 0.25
        # publish steering angle
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = Kp3* steering_angle
        msg.drive.speed = velocity
        self.drive_pub.publish(msg)

        # publish errors
        msg_error = parking_error()
        msg_error.x_error = desired_distance * cos(theta)
        msg_error.y_error = desired_distance * sin(theta)
        msg_error.distance_error = desired_distance
        self.error_pub.publish(msg_error)


if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
