#!/usr/bin/env python
import rospy
import numpy as np
import time
import utils
import tf
import threading

from geometry_msgs.msg import PoseArray, PoseStamped, Point
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):


        self.VISUALIZATION = True

        self.lock = threading.RLock()


        self.scale = 1.5

        self.approach = False
        self.approach_speed = 11.0
        self.approach_dist = 10.0

        self.odom_topic = rospy.get_param("~odom_topic") #/tesse/odom for TESSE #/odom
        self.speed = 20.5 # RVIZ CURRENTLY 50 sec: 15.0, 1.5
        speed_scale = 1.0
        # self.lookahead = speed_scale*self.speed

        self.lookahead = 1.5*15

        self.last_seg = False



        # self.wrap             = False # FILL IN #
        self.wheelbase_length = 0.30 # FILL IN #
        self.segs_index = 0
        self.segs = None
        self.min_index = 0


        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        rospy.loginfo('INITIALIZED')
        print(self.odom_topic)
        print(self.trajectory)
        self.stop_point = None
        self.parking_distance = 0.25
        self.traj_received = False

        self.segs = None



        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        # self.traj_sub = rospy.Subscriber("/loaded_trajectory/path", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)

        self.tf_listener = tf.TransformListener()

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)

        # self.collision_sub = rospy.Subscriber("/tesse/collision",)

        self.global_position = None

        self.marker_pub = rospy.Publisher("lookahead_marker", PoseStamped, queue_size=1)
        self.gp_pub = rospy.Publisher("global_position", Marker, queue_size=1)
        self.ps_pub = rospy.Publisher("global_ps", PoseStamped, queue_size=1)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        # self.trajectory.publish_viz(duration=0.0)
        self.stop_point = None
        self.traj_received = True
        self.min_index = 0
        print(self.traj_received)
#here
        points = self.trajectory.points

        seg1 = np.array(points[:-1])
        seg2 = np.array(points[1:])

        self.segs = np.hstack((seg1, seg2))

        print(self.segs)
#here




    def odom_callback(self, msg):
        """
        """
        # print("IN TOP ODOM CB")
        # rospy.loginfo("IN ODOM CB TOP")
        start = time.time()
        with self.lock:

            if not self.traj_received:
                return

            # print('IN ODOM CB')
            odom_world = PoseStamped()
            odom_world.header = msg.header
            odom_world.pose = msg.pose.pose
            transformed = self.tf_listener.transformPose("/map", odom_world)

            self.global_position = transformed.pose

            # self.global_position = msg.pose.pose

            # x_global, y_global = self.global_position.position.x, self.global_position.position.y
            #
            # if self.VISUALIZATION:
            #
            #
            #     #sepearte marker
            #     marker = PoseStamped()
            #     marker.header.frame_id = "/map"
            #     marker.header.stamp = rospy.Time.now()
            #
            #     marker.pose.orientation.x = self.global_position.orientation.x
            #     marker.pose.orientation.y = self.global_position.orientation.y
            #     marker.pose.orientation.z = self.global_position.orientation.z
            #     marker.pose.orientation.w = self.global_position.orientation.w
            #     marker.pose.position.x = float(x_global)
            #     marker.pose.position.y = float(y_global)
            #     marker.pose.position.z = 0.0
            #
            #     self.ps_pub.publish(marker)



            # points = self.trajectory.points
            #
            # seg1 = np.array(points[:-1])
            # seg2 = np.array(points[1:])
            #
            # segs = np.hstack((seg1, seg2))
            # print('SEGMENTS', segs, segs.shape)

            # segs = segs[self.segs_index:, :]



            # distances = np.apply_along_axis(self.closest_point, 1, segs)
            # print('DISTANCES: ', distances)

            # min_index = np.argmin(distances)


            # self.segs = self.segs[min_index:, :]

            # self.segs_index += min_index

            min_index = 0

            # self.find_lookahead(segs)
            self.find_lookahead()

        end = time.time() - start

        # print('ODOM CALLBACK TIME', end)

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

        """
        np.argmin over a vector of distances
        use index to find actual point.
        """

    def find_lookahead(self):
        """
        also transform to vehicle corodinates
        TO-DO:
        -find point
        -transform point to vehicle coordinates (subtract then apply rotation matrix)
        """
        # print('IN FIND LOOKAHEAD')
        # self.index_history.add(min_index)

        x_global, y_global = self.global_position.position.x, self.global_position.position.y

        # print('GLOBAL VALUES:', x_global, y_global)


        x_circle, y_circle = None, None

        self.approach = False

        if self.stop_point is not None:
            x_circle, y_circle = self.stop_point
        else:
            for x1, y1, x2, y2 in self.segs[self.min_index:, :]:

                if self.min_index == len(self.segs) - 1:
                    self.last_seg = True


                # print('MIN INDEX: ', self.min_index)

                Q = np.array([x_global,y_global])
                r = self.lookahead

                P1 = np.array([x1,y1])
                P2 = np.array([x2,y2])
            	V = P2 - P1

            	a = np.sum(V * V)
            	b = 2 * np.sum(V * (P1 - Q))
            	c = np.sum(P1 * P1) + np.sum(Q * Q) - 2 * np.sum(P1 * Q) - r**2
            	disc = b**2 - 4 * a * c

            	if disc < 0:
                    self.min_index += 1
                    continue

            	sqrt_disc = disc ** 0.5
            	t1 = (-b + sqrt_disc) / (2.0 * a)
            	# t2 = (-b - sqrt_disc) / (2.0 * a)

            	if not (0.0 <= t1 <= 1.0):# or 0.0 <= t2 <= 1.0):
                    self.min_index += 1
            	    continue

                # t = max(t1)
                target = P1 + t1 * V
                x_circle, y_circle = target[0], target[1]

                # dist_temp = np.linalg.norm(P2 - target)

                if np.linalg.norm(P2 - target) <= self.approach_dist:
                    self.approach = True

                if np.linalg.norm(target - P1) <= self.approach_dist:
                    self.approach = True


                # print('TARGET: ',target)
                # print('TARGET POINTS: ',t1)
                break

        quat = (self.global_position.orientation.x, self.global_position.orientation.y,
                self.global_position.orientation.z, self.global_position.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        theta = euler[2]



        rotation = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])

        rotated_point = rotation.dot(np.array([[x_circle-x_global], [y_circle-y_global]]))

        x_rf, y_rf = rotated_point[0], rotated_point[1]

        # print('YAW YEEET: ', (x_global, y_global), (x_circle, y_circle),(x_rf,y_rf))

        if self.VISUALIZATION:
            marker = PoseStamped()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()

            marker.pose.orientation.x = self.global_position.orientation.x
            marker.pose.orientation.y = self.global_position.orientation.y
            marker.pose.orientation.z = self.global_position.orientation.z
            marker.pose.orientation.w = self.global_position.orientation.w
            marker.pose.position.x = float(x_circle)
            marker.pose.position.y = float(y_circle)
            marker.pose.position.z = 0.0

            self.marker_pub.publish(marker)



        alpha = np.arctan2(y_rf,x_rf)
        curvature = np.arctan2(2.0*self.wheelbase_length*np.sin(alpha)/self.lookahead, 1.0)/self.scale

        # shift = np.sqrt((x_rf - self.wheelbase_length/2)**2 + y_rf**2)
        # curvature = 2.0*y_rf/(shift**2.0)
        # if self.stop_point is not None:
        #     if np.sqrt((x_global - self.stop_point[0])**2 + (y_global - self.stop_point[1])**2)  < self.parking_distance:
        #         speed = 0
        #     else:
        #         speed = self.speed
        # else:

        if self.last_seg:
            speed = 300.0
            print('SPEEEEEEEEEEED')

        elif self.approach:
            speed = self.approach_speed
            print('APPROACH SPEED')

        else:
            speed = self.speed

        # print('ALPHA, wheel, look:', alpha, self.wheelbase_length, self.lookahead)
        # print('LOOKAHEAD POINT: ', x_rf, y_rf)
        # print('CURVATURE: ', curvature)

        # curvature = -1.0
        # speed = 1.0 #testing only!!!!!!!

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.drive.steering_angle = curvature
        drive_msg.drive.speed = speed
        # print('PUBLISHING DRIVE MSG: ', drive_msg)
        self.drive_pub.publish(drive_msg)

'''
steps:
1. get current location of car
2. find the path point closest to the vehicle
3. find lookahead point --> point on the circle on the line segment with the smallest index
4. get lookahead point in vehicle coordinates
5. calc curvature to that point and set steering angle to that curvature


publish angle function (step 5)

trajectory current is a collection of poses
'''


if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    rospy.loginfo('IT HAS BEEN LAUNCHED')
    pf = PurePursuit()
    rospy.spin()


    '''
                for x1, y1, x2, y2 in segs[min_index:, :]: #segment in global coords

                    if x2 == segs[-1, 2] and y2 == segs[-1, 3]:
                        x_circle, y_circle = x2, y2
                        self.stop_point = x_circle, y_circle
                        break

                    a = (x2 - x1)**2.0 + (y2 - y1)**2.0 #ballin
                    b = 2.0 * ((x2 - x1) * (x1 - x_global) + ((y2-y1) * (y1 - y_global))) #chillin
                    c = (x1**2.0 + y1**2.0 + x_global**2.0 + y_global**2.0 ) - 2.0 * (x1*x_global + y1*y_global) - float(self.lookahead)**2.0

                    disc = b**2.0 - 4.0 * a * c

                    if disc < 0.0:
                        # print('DISC < 0')
                        continue

                    sqrt_disc = np.sqrt(disc)
                    t1 = (-b + sqrt_disc) / (2.0*a)
                    t2 = (-b - sqrt_disc) / (2.0 * a)

                    if not(0 <= t1 <= 1 or 0 <= t2 <= 1):
                        continue

                    # rospy.logerr('HAHA GOOD MORNING:' + str(t1) + ' ' + str(t2))


                    t = max(t1, t2)

                    x_circle, y_circle = x1 + (t*(x2 - x1)), y1 + (t*(y2 - y1))

                    break
    '''
