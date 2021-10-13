#!/usr/bin/env python2

import rospy
import numpy as np
import time
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from visualization_msgs.msg import Marker


class ParticleFilter:

    def __init__(self):
        # Get parameters'

        self.lock = threading.RLock()
        self.n_particles = rospy.get_param('num_particles', 500)
        self.particles = None
        self.prob_out = None
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        self.prev_time = None
        self.curr_time = None
        self.VISUALIZATION = True

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")

        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.lidar_callback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odom_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        #/followed_trajectory/start_point

        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_init_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        # publishers for different topics
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)

        self.posearray_pub = rospy.Publisher("/posearray", PoseArray, queue_size=1)

        self.marker_pub = rospy.Publisher("avg_marker", Marker, queue_size=1)

        # Initialize the models

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz


        # Publish a transformation frame between the map
        # and the particle_filter_frame.


    def pose_init_callback(self, data):
        """
        At initial initialization (when 2D pose estimate is used), creates an
        array of particles that will be refined to follow location of the car.

        args:
            pose  (from 2d pose estimate)

        returns:
            nothing but sets self.particles for the first time
        """
        with self.lock:
            self.init_pos = data.pose.pose.position
            init_quar = data.pose.pose.orientation
            x,y,z,w = init_quar.x, init_quar.y, init_quar.z, init_quar.w
            self.init_angle = euler_from_quaternion([x,y,z,w])[2]

            # making particles with noise
            std = 0.5
            xs = np.random.normal(loc = self.init_pos.x, scale = std, size = (self.n_particles,1))
            ys = np.random.normal(loc = self.init_pos.y, scale = std, size = (self.n_particles,1))
            thetas = np.random.normal(loc = self.init_angle, scale = std, size = (self.n_particles,1))

            self.particles = np.hstack((xs, ys, thetas))
            x_avg, y_avg, theta_avg = self.publish_average(self.particles, self.prob_out)

            # Visualization!
            if self.VISUALIZATION:
                self.posearray_msg = PoseArray()
                self.posearray_msg.header.frame_id = "/map"
                self.posearray_msg.header.stamp = rospy.Time.now()
                self.posearray_msg.poses = []
                for r in range(self.particles.shape[0]):
                    x = self.particles[r][0]
                    y = self.particles[r][1]
                    th = self.particles[r][2]
                    quat = quaternion_from_euler(0, 0, th)
                    new_pose = Pose(Point(x,y,0),Quaternion(*quat))
                    self.posearray_msg.poses.append(new_pose)
                # rospy.loginfo('[INIT]: %s', self.particles.shape)
                self.posearray_pub.publish(self.posearray_msg)

    def odom_callback(self, data):
        """
        Given odomotry data from odom_topic, update particles with probable next
        location after odometry.
        """
        start = time.time()
        with self.lock: # lock
            if self.particles is not None:
                self.curr_time = data.header.stamp.to_sec()

                if self.prev_time is None:
                    self.prev_time = self.curr_time
                else:
                    time_delta = self.curr_time - self.prev_time

                    # Use motion model and odometry in order to calculate probable locations
                    # account for time step between functions
                    odom = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z])*time_delta #dx, dy, dtheta
                    self.particles = self.motion_model.evaluate(self.particles, odom)

                    # publish new average values
                    x_avg, y_avg, th_avg = self.publish_average(self.particles, self.prob_out)

                    #visualization
                    if self.VISUALIZATION:
                        self.posearray_msg = PoseArray()
                        self.posearray_msg.header.frame_id = "/map"
                        self.posearray_msg.header.stamp = rospy.Time.now()
                        self.posearray_msg.poses = []
                        for r in range(self.particles.shape[0]):
                            x = self.particles[r][0]
                            y = self.particles[r][1]
                            th = self.particles[r][2]
                            quat = quaternion_from_euler(0, 0, th)
                            new_pose = Pose(Point(x,y,0),Quaternion(*quat))
                            self.posearray_msg.poses.append(new_pose)

                        self.posearray_pub.publish(self.posearray_msg)

                    # update time
                    self.prev_time = self.curr_time

                    # rospy.loginfo('[ODOM CALLBACK]: %s', self.particles)
                    # rospy.loginfo('[ODOM CALLBACK]: %s, %s', np.max(self.particles,axis=0), np.min(self.particles,axis=0))

            else:
                pass
                # rospy.logerr("SELF.PARTICLES IS NONE OH NO!")
        # rospy.loginfo("[ODOM TIME]: %s", round(time.time()-start,5))

    def lidar_callback(self, data): #slice
        """
        Subscribes to lidar scans and uses sensor model in order to compute
        probabilities for likelihood of accuracy. Resamples particles based on
        those probabilities.

        args:
            init values, and lidar scan data.

        Returns:
            no return, but updates self.particles and self.prob_out
        """
        start = time.time()
        with self.lock: # lock in case of conflicts

            # ensure particles has been initialized by self.pose_init_callback
            if self.particles is not None:
                observation = np.array(data.ranges)

                # evaluates prob out using sensor table and normalizes
                self.prob_out = self.sensor_model.evaluate(self.particles, observation)
                self.prob_out = self.prob_out/np.sum(self.prob_out)

                # resamples based on self.prob_out
                particles_indices = np.arange(self.n_particles)
                resampled_indices = np.random.choice(particles_indices,self.n_particles,p = self.prob_out)
                self.particles = self.particles[resampled_indices,:]

                # computes averages of x y and theta
                x_avg, y_avg, theta_avg = self.publish_average(self.particles, self.prob_out)

                # visualizes pose array
                if self.VISUALIZATION:
                    self.posearray_msg = PoseArray()
                    self.posearray_msg.header.frame_id = "/map"
                    self.posearray_msg.header.stamp = rospy.Time.now()
                    self.posearray_msg.poses = []

                    for r in range(self.particles.shape[0]):
                        x = self.particles[r][0]
                        y = self.particles[r][1]
                        th = self.particles[r][2]
                        quat = quaternion_from_euler(0, 0, th)
                        new_pose = Pose(Point(x,y,0),Quaternion(*quat))
                        self.posearray_msg.poses.append(new_pose)
                    # rospy.loginfo('[LIDAR CALLBACK]: %s, %s', np.max(self.particles,axis=0), np.min(self.particles,axis=0))
                    self.posearray_pub.publish(self.posearray_msg)

            else:
                pass
                # rospy.logerr("SELF.PARTICLES IS NONE OH NO!")
        # rospy.loginfo("[LIDAR TIME]: %s", round(time.time()-start,5))

    def publish_average(self, particles, prob_out): #look at implementing exp value instead
        """
        Helper function that uses particles and probabilities in order to compute
        either expected value or average. Publishes odometry data to odom topic

        args:
            particles and probability

        returns:
            tuple of acerage values
        """

        # SPLIT PARTICLES
        x_vals = particles[:, 0]
        y_vals = particles[:, 1]
        theta_vals = particles[:, 2]

        # if no prob has been computed, then expected value of x and y
        if prob_out is not None:
            x_avg = np.dot(x_vals,prob_out)
            y_avg = np.dot(y_vals,prob_out)

        # else, mean
        else:
            x_avg = np.mean(x_vals)
            y_avg = np.mean(y_vals)

        # average of circular values using cos and sin
        cos_val = np.mean(np.cos(theta_vals))
        sin_val = np.mean(np.sin(theta_vals))

        if cos_val > 0 and sin_val > 0:
            theta_avg = np.arctan(sin_val/cos_val)
        elif cos_val < 0:
            theta_avg = np.arctan(sin_val/cos_val) + np.pi
        else:
            theta_avg = np.arctan(sin_val/cos_val) + (2.0 * np.pi)

        # compute quaternion
        quat_avg = quaternion_from_euler(0, 0, theta_avg)

        # marker publishing
        if self.VISUALIZATION:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.POINTS

            marker.action = Marker.ADD
            marker.pose.orientation.w =0.0
            marker.pose.position.x = x_avg
            marker.pose.position.y = y_avg
            marker.pose.position.z = 0.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.points = []
            t = rospy.Duration(0)
            point = Point()
            point.x = x_avg
            point.y = y_avg
            point.z = 0.0

            marker.points.append(point)
            self.marker_pub.publish(marker)

        # odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = "/map"
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.pose.pose = Pose(Point(x_avg, y_avg, 0), Quaternion(*quat_avg))
        self.odom_pub.publish(odom_msg)

        # transforms to particle_filter_frame
        self.publish_transform([x_avg, y_avg, theta_avg])
        # rospy.loginfo("%s, %s", x_avg, y_avg)

        return x_avg, y_avg, theta_avg

    def publish_transform(self, average_pose):
        """
        Given average pose, transforms to particle filter frame using a
        broadcaster.
        """
        br = tf.TransformBroadcaster()
        br.sendTransform((average_pose[0], average_pose[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, average_pose[2]),
                        rospy.Time.now(),
                        self.particle_filter_frame,
                        "/map")


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
