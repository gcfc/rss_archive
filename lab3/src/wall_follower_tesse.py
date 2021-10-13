#!/usr/bin/env python2
import numpy as np
import tf
import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from math import pi, sqrt
import threading


class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/tesse/front_lidar/scan"
    DRIVE_TOPIC = "/tesse/drive"
    VELOCITY = 20
    DESIRED_DISTANCE = 5.0

    lock = threading.RLock()


    err_history = []
    err_history_size = 25

    min_dist_history = []
    min_dist_history_size = 10

    total_history = []

    angle_history = []
    boundary_dist = 4.0

    right_bound = np.array([197.3412,-237.2167, 273,-221])
    left_bound = np.array([195.87,-223.936, 280.365,-204.0482])
    # right_bound = np.array([-6.894,2.606,-100.304,-19.712])
    # left_bound = np.array([-2.335,-15.686,-97.252,-36.302])

    true_dir = 0.23
    steering_angle = 0.0
    disturbance = 0.0
    odom_dir = None
    prev_steering_angle = 0.0

    window_scale = 16

    global_position = None

    end_point = np.array([284,6389, -215.8704])
    # end_point = np.array([-100.200, -29.420])
    # end_point = np.array([-102.0, -29.8])

    def __init__(self):
        print('Starting Up')
        # self._line_marker.points=[]
        self.tf_listener = tf.TransformListener()
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=50)
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)
        self.odom_topic = rospy.get_param("~odometry_topic")
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)


    def odom_cb(self, data):
        transformed_pose = data.pose.pose
        quat = (transformed_pose.orientation.x, transformed_pose.orientation.y,
                transformed_pose.orientation.z, transformed_pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        self.odom_dir = euler[2]

        # self.global_position = transformed_pose
        odom_world = PoseStamped()
        odom_world.header = data.header
        odom_world.pose = data.pose.pose
        transformed = self.tf_listener.transformPose("/map", odom_world)

        self.global_position = transformed.pose

    def lidar_slice(self, scans):
        window_size = len(scans)//self.window_scale
        averages = []

        for i in range(0, len(scans) - window_size):
            window  = scans[i:i+window_size]

            std = np.std(window)
            avg = np.mean(window)
            i = ((window>(avg-2*std)) & (window<(avg+2*std)) )#get indices
            avg = np.mean(window[i])
            averages.append(avg)

        max_ind = np.argmax(averages)
        # print('AVG LEN: ', len(averages))
        return max_ind

    def closest_point(self, segment):
        """
        To do:
        figure out way to use numpy vectors and be fast
        """
        # print('CLOSEST POINT SEGMENT', segment)
        x1, y1, x2, y2 = segment[0], segment[1], segment[2], segment[3]
        # print('XYZ: ',self.global_position.position)
        x,y,z = self.global_position.position.x, self.global_position.position.y, self.global_position.position.z

        A = x - x1
        B = y - y1
        C = x2 - x1
        D = y2 - y1

        dot = A * C + B * D
        len_sq = C * C + D * D

        param = -1.0

        if len_sq != 0:
            param = dot / len_sq

        if param < 0:
            xx = x1
            yy = y1

        elif param > 1:
            xx = x2
            yy = y2

        else:
            xx = x1 + param * C
            yy = y1 + param * D

        dx = x - xx
        dy = y - yy
        distance = np.sqrt(dx * dx + dy * dy)

        return distance

    def purely_pursue(self, lookahead):
        x_global, y_global = self.global_position.position.x, self.global_position.position.y

        quat = (self.global_position.orientation.x, self.global_position.orientation.y,
                self.global_position.orientation.z, self.global_position.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        theta = euler[2]



        rotation = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])

        rotated_point = rotation.dot(np.array([[self.end_point[0]-x_global], [self.end_point[1]-y_global]]))

        x_rf, y_rf = rotated_point[0], rotated_point[1]

        alpha = np.arctan2(y_rf,x_rf)
        curvature = np.arctan2(2.0*0.30*np.sin(alpha)/lookahead, 1.0)/2.0

        return curvature


    def callback(self, data):

        if self.global_position is None:
            return


        angle_min = data.angle_min
        angle_max = data.angle_max
        angle_increment = data.angle_increment
        scan_ranges = np.array(data.ranges)


        size = len(scan_ranges)
        angles = np.array([angle_min + i*angle_increment for i in range(0,size)])

        # scan_ranges = scan_ranges[::3]
        # angles = angles[::3]

        # front_ranges = scan_ranges #[size//6:5*size//6]

        scan_ranges = scan_ranges[3*size//8:5*size//8]
        # print(scan_ranges)
        # scan_ranges[scan_ranges > 50.0] = 50.0
        window_size = len(scan_ranges)//self.window_scale

        angles = angles[3*size//8:5*size//8]

        scan_ranges = scan_ranges[::5]
        angles = angles[::5]

        dist_to_end = np.linalg.norm(np.array([self.global_position.position.x - self.end_point[0],self.global_position.position.y - self.end_point[1]]))

        right_dist = self.closest_point(self.right_bound)
        # print('RIGHT DIST: ',right_dist)

        left_dist = self.closest_point(self.left_bound)
        # print("LEFT DIST: ",left_dist)
        buffer = 0.0


        right = False
        left = False

        if right_dist <= self.boundary_dist:# and dist_to_end <= 20.0:
            # print('SCAN RANGES:', len(scan_ranges))
            # scan_ranges = scan_ranges[:2*len(scan_ranges)//5]
            # angles = angles[:2*len(scan_ranges)//5]
            # buffer = 1.0
            right = True
            # print('TOO CLOSE RIGHT')

        if left_dist <= self.boundary_dist:# and dist_to_end <= 15.0:
            # scan_ranges = scan_ranges[3*len(scan_ranges)//5:]
            # angles = angles[3*len(scan_ranges)//5:]
            # buffer = -1.0
            left = True
            # print('TOO CLOSE LEFT')



        # print('SCAN RANGES LEN: ', len(angles))
        max_ind = self.lidar_slice(scan_ranges)

        # print('MAX IND:', max_ind)
        if right or left:
            target_angle = 1.2*self.purely_pursue(dist_to_end)
            # print("target curvature")
        # elif left:
        #     target_angle = -1.0

        elif dist_to_end <= 20.0:
            # print('TRYING SOMETHING HERE')
            with self.lock:
                target_angle = self.purely_pursue(dist_to_end)
                # print('CURVATURE')

        else:
            try:
                # print('trying')
                target_angle = 0.95*angles[max_ind] + buffer# + int(window_size/2)] #+ buffer
                # print("normal curvature")
            except:
                # print('except part')
                target_angle = 1.2* self.purely_pursue(dist_to_end/1.5)

        # target_ang


        # print('TARGET',target_angle)
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.drive.steering_angle = target_angle
        msg.drive.speed = self.VELOCITY
        self.pub.publish(msg)

        self.prev_steering_angle = self.steering_angle

        # elif right_dist <= self.boundary_dist and dist_to_end <= 15.0:
        #     # print('SCAN RANGES:', len(scan_ranges))
        #     scan_ranges = scan_ranges[len(scan_ranges)//2:]
        #     angles = angles[len(scan_ranges)//2:]
        #     target_angle = 0.6
        #     print('TOO CLOSE RIGHT')
        #
        # elif left_dist <= self.boundary_dist and dist_to_end <= 15.0:
        #     scan_ranges = scan_ranges[:3*len(scan_ranges)//4]
        #     angles = angles[:3*len(scan_ranges)//4]
        #     target_angle = -0.6
        #     print('TOO CLOSE LEFT')

        # self.angle_history.append(target_angle)




        # min_dist = np.min(front_ranges)
        # min_dist_angle = angles[np.argmin(front_ranges)]
        #
        # if min_dist < self.DESIRED_DISTANCE:
        #     # aggressively turn
        #     IMPULSE = np.pi / 2.0
        #     if min_dist_angle < 0:
        #         self.disturbance = IMPULSE * np.cos(min_dist_angle)
        #     else:
        #         self.disturbance = -IMPULSE * np.cos(min_dist_angle)
        #
        # # PID control with true_dir here
        #
        # # self.disturbance = self.disturbance/
        #
        # kp = 1.75
        # kd = 4.7
        # ki = 1.0
        #
        # steering_error = self.true_dir - self.odom_dir # DRIVE STRAIGHT DANG IT
        #
        # error_velocity = self.steering_angle - self.prev_steering_angle #for kd
        #
        # # for ki
        # self.err_history.append(steering_error)
        # if len(self.err_history) > self.err_history_size:
        #     self.err_history.pop(0)
        # err_int = -np.sum(self.err_history) * (1/25.0)
        #
        # self.steering_angle = kp*steering_error + kd*error_velocity + ki*err_int + self.disturbance
        # # self.steering_angle = max(min(self.steering_angle, 2*pi/9), -2*pi/9)

        # rospy.loginfo("%s, %s, %s", kp*steering_error, kd*error_velocity, ki*err_int)
        # rospy.loginfo(self.steering_angle)

        # target_angle = -1.0





    def least_squares(self, x, y):
        # original least squares code
        A = np.vstack([x, np.ones(len(x))]).T
        m,b = np.linalg.lstsq(A,y,rcond = None)[0]
        return m,b


if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
