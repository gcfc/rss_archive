#!/usr/bin/env python2

import random
import numpy as np
import rospy

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        #self.noise = np.array([random.gauss(0,5),])
        self.initial_pose = [1.75,1.25,3.14]
        self.deterministic = rospy.get_param('~deterministic',True)
        self.mean = 0
        self.std = 0.05
        #self.initial_pose = None
        #once initalized, 3x1 [dx dy dtheta]
        #[1.75,1.25,3.14]
        ####################################

    def set_initial_pose(self,initial_pose):
        self.initial_pose = initial_pose

    def R(self,row):
        theta= -1.0*row[2]
        return np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta),0],[0,0,1]])



    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        # particles in world frame
        # odometry in robot frame
        assert self.initial_pose is not None

        def compute_new(row):
            """
            Given a row, computes a new row for its probable state given odeometry.

            args:
                row: a particle

            returns new particle
            """
            theta = -1.0*row[2]
            rm = np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta),0],[0,0,1]])
            delta_xw = odometry.dot(rm)
            #row += delta_xw
            return (row+delta_xw)

        # changes all the particles
        particles = np.apply_along_axis(compute_new, 1, particles)


        # self.deterministic = ground truth, no noise.
        if self.deterministic:
            return particles

        #add Gaussian noise along each axis
        particles[:,0] += np.random.normal(loc=0,scale=0.02,size=particles.shape[0])
        particles[:,1] += np.random.normal(loc=0,scale=0.02,size=particles.shape[0])
        particles[:,2] += np.random.normal(loc=0,scale=0.1,size=particles.shape[0])


        return particles

        ###################################
