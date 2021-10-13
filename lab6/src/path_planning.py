#!/usr/bin/env python

import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import rospkg
import time, os
from utils import LineTrajectory
from math import *
import sys
import random
# np.set_printoptions(threshold=sys.maxsize)
import heapq
import matplotlib.pyplot as plt
from scipy import ndimage


class Node:
    def __init__(self, x, y): # cost?
        self.x = x
        self.y = y
        # self.cost?

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __repr__(self):
        return 'Node '+str((self.x,self.y))

    def __hash__(self):
        return hash((self.x,self.y))

    def __ne__(self, other):
        return self.x != other.x or self.y != other.y

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.tf_listener = tf.TransformListener()

        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.ps_pub = rospy.Publisher("transformed_odom", PoseStamped, queue_size=1)
        self.cp_sub = rospy.Subscriber("/clicked_point", PointStamped, self.cp_cb, queue_size=1)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback, queue_size=1)

        self.graph = dict()
        self.parent_dict = dict()
        self.all_nodes = set()
        self.start = None
        self.goal = None
        self.global_goal = None
        self.fake_goal = None
        self.map = None
        self.map_height = None
        self.map_width = None
        self.origin = None
        self.res = None
        # self.graph = None
        self.cost = dict()
        self.global_pose = None
        self.global_time = None
        self.global_start = None
        self.t = 0.1
        self.replan_distance = 2.5
        self.sample_rate = 0.7

        rospy.loginfo('LET\'S GET GOIN BABY')

    '''
    GAME PLAN
    parse OccupancyGrid map into grid
    grid to nodes (BIG dict ENERGY)
    write A* code (find on stackoverflow or sth)
    path planning with dubins curve
    visualize traj using path (see readme)
    '''

    def pixel_to_actual(self, pixel): #REWRITE THIS
        theta = self.origin[3]


        # offset_x = self.origin[0]
        # offset_y = self.origin[1]
        # x = (offset_x - x)
        # y = y - self.map_height*self.res + offset_y

        # return (x, y)


        px, py = pixel # v, u

        x = px*self.res
        y = py*self.res

        rotated_x = cos(theta)*x - sin(theta) * y # -x
        rotated_y = sin(theta)*x + cos(theta) * y # -y

        actual_x = rotated_x + self.origin[0]
        actual_y = rotated_y + self.origin[1]
        return actual_x, actual_y



    def actual_to_pixel(self, pose): #REWRITE THIS
        theta = self.origin[3]


        # px = int((self.origin[0] - x)/self.res)
        # py = int((y + self.map_height*self.res - self.origin[1])/self.res)

        # return (px, py) # v, u

        x, y = pose

        translated_x = x - self.origin[0]
        translated_y = y - self.origin[1]

        inv_rot_mat = np.linalg.inv(np.array([[cos(theta), -sin(theta)], [sin(theta), cos(theta)]]))
        rotated_x , rotated_y = inv_rot_mat.dot(np.array([[translated_x],[translated_y]])) # - translated_x
        # rotated_y = -cos(theta) * translated_x + sin(theta) * translated_y # - translated_y

        pixel_x = rotated_x/self.res
        pixel_y = rotated_y/self.res

        return int(pixel_x), int(pixel_y)

    def cp_cb(self, msg):
        print(self.actual_to_pixel((msg.point.x, msg.point.y)))

    def get_neighbors(self, node):
        neighbors = set()
        possible_x, possible_y = [node.x+i for i in range(-1,2,1)],[node.y+i for i in range(-1,2,1)]
        possible_nodes = [Node(x,y) for x in possible_x for y in possible_y if 0 <= x < self.map_width and 0 <= y < self.map_height]

        for n in possible_nodes:
            if self.map[n.y, n.x] == 0 and n != node:
                neighbors.add(Node(n.x, n.y))
                # print('GOT HERE BRO')
        return neighbors

    def get_distance(self, node1, node2):
        return sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)

    def get_cost(self, curr, neighbor):
        return self.cost[curr] + self.get_distance(curr, neighbor)

    def get_heuristic(self, node):
        return self.get_distance(node, self.goal)

    def new_random_point(self):
        """
        Initialize a new float point
        """
        if random.randint(0,100) > self.sample_rate:
            new_point = (random.uniform(-104.0, -5.0), random.uniform(-41.0, 2.5))
        else:
            new_point = tuple(self.fake_goal)
        return new_point
        # return (random.uniform(-104.0, -5.0), random.uniform(-41.0, 2.5))



    def find_nearest(self, node):
        """
        some code to find nearest
        """

        min = np.Inf
        min_node = None
        for child in self.all_nodes:
            distance = np.linalg.norm(np.array([node[0] - child[0], node[1] - child[1]]))
            if distance < min:
                min = distance
                min_node = child
        self.all_nodes.add(node)

        if min_node is None:
            return node

        return min_node

    def find_neighbors_radius(self, node, radius):
        """
        find all neighbors a radius away
        """
        neighbors = set()
        possible_x, possible_y = [node.x+i for i in range(-radius,radius + 1,1)],[node.y+i for i in range(-radius,radius + 1,1)]
        possible_nodes = [Node(x,y) for x in possible_x for y in possible_y if 0 <= x < self.map_width and 0 <= y < self.map_height]

        for n in possible_nodes:
            if self.map[n.y, n.x] == 0 and n != node and np.linalg.norm(np.array([node.x, node.y]) - np.array([n.x, n.y])) <= radius:
                neighbors.add(Node(n.x, n.y))

        return neighbors

    def return_path(self, goal):
        for k, v in self.parent_dict.items():
            if k == v:
                print('DUPLICATE')
                if k == self.fake_goal:
                    print('DUPLICATE FAKE')
                elif k == self.global_start:
                    print('DUPLICATE START')
                else:
                    print('None of those, RANDOM POINT')
        return
        path = [goal]

        while path[-1] is not self.global_start:

            path.append(self.parent_dict[path[-1]])
            print('PATH', path)

        return path.reverse()


        # child = goal
        # path = [child]
        # self.graph[goal] = list()
        # while child is not self.global_start:
        #
        #     for node in self.graph.keys():
        #         if self.graph[node] is None:
        #             continue
        #
        #         if tuple(child) in self.graph[node]:
        #             path.append(node)
        #             child = node
        #             break
        #
        # path.reverse()
        #
        # return path



    def rrt_star_search(self):
        """
        To -Do: function that gives us a random Point
        function that finds the distance -> have already
        function that determines if it is an obstacle or not
        cost function --> have already


        """


        counter = 0
        num_tries = 100
        self.graph = dict()
        while counter < num_tries:
            random_point = self.new_random_point()
            random_square = Node(*self.actual_to_pixel(random_point))

            # convert random point to pixel
            if self.map[random_square.y, random_square.x] != 0:
                continue

            nearest_point = self.find_nearest(random_point)
            if nearest_point is None:
                print("NEAREST POINT IS NONE")
            if nearest_point == random_point:
                print('this is happeninga lot')
                if random_point == self.fake_goal:
                    print('this is happening a lot and duplicate fake')
                continue


            if nearest_point not in self.graph.keys():
                self.graph[nearest_point] = [random_point]
                self.parent_dict[random_point] = nearest_point
            else:
                self.graph[nearest_point].append(random_point)
                self.parent_dict[random_point] = nearest_point

            if np.linalg.norm(np.array([self.fake_goal[0] - random_point[0], self.fake_goal[1] - random_point[1]])) < 0.1:
                if random_point != self.fake_goal:
                    self.graph[random_point] = [tuple(self.fake_goal)]
                    self.parent_dict[self.fake_goal] = random_point
                    print('PATH HAS BEEN SUCCESSFULLY BUILT')
                    return self.graph

        print('NEVER GETS THERE')
        return self.graph

        # radius = 5
        # self.map
        #
        # num_tries = 100
        #
        # self.graph = dict()
        #
        # for i in range(0, num_tries):
        #     random_point = self.new_random_point()
        #
        #     #need to convert to pixel ->
        #     if self.map[random_point.y, random_point.x] != 0:
        #         continue
        #
        #     nearest_point = self.find_nearest(self.graph, random_point)
        #     random_cost = self.get_distance(random_point, nearest_point)
        #     best_point, neighbors = self.find_neighbors(self.map, random_point, radius)
        #
        #     self.cost[random_point] = random_cost
        #
        #     link = self.connect(random_point, best_point)
        #

            # #rewiring
            # for neighbor in neighbors:
            #     new_cost =  self.get_cost(curr, neighbor)
            #
            #     if random_cost + self.get_distance(random_point, neighbor) < self.cost[neighbor]:
            #         self.cost[neighbor] = random_cost + self.get_distance(random_point, neighbor)
            #         self.parent[neighbor] = random_point
            #
            #
            #     if neighbor not in self.cost or new_cost < self.cost[neighbor]:
            #         pass


    def lidar_callback(self, data):
        '''
        update map based on lidar data
        '''

        if self.global_time is None or (time.time() - self.global_time) > 3: # do this every 3 seconds
            angle_min = data.angle_min
            angle_max = data.angle_max
            angle_increment = data.angle_increment
            # scan_ranges = np.array(data.ranges)
            scan_ranges = np.array([data.ranges[i] for i in range(0,len(data.ranges),3)])
            size = len(scan_ranges)

            # angles = np.array([angle_min + i*angle_increment for i in range(0,size)])
            angles = np.array([angle_min + i*angle_increment for i in range(0,size,3)])
            cosine = np.cos(angles)
            sine = np.sin(angles)

            x_comp = np.multiply(scan_ranges, cosine)
            y_comp = np.multiply(scan_ranges, sine)

            x = x_comp[size//6:5*size//6]
            y = y_comp[size//6:5*size//6]
            points = np.array(list(zip(x,y)))
            # dists = scan_ranges[size//6:5*size//6]

            quat = (self.global_pose.orientation.x, self.global_pose.orientation.y,
                    self.global_pose.orientation.z, self.global_pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quat)
            theta = euler[2]

            rotation = np.array([[np.cos(-theta), np.sin(-theta)], [-np.sin(-theta), np.cos(-theta)]])
            world_points = rotation.dot(points.T) + np.array([[self.global_pose.position.x], [self.global_pose.position.y]])
            pixel_points = np.zeros(world_points.shape)

            #convert to pixel
            for i in range(world_points.shape[0]):
                _node = self.actual_to_pixel(*world_points[:,i])
                pixel_points[:,i] = np.array([_node.x, _node.y]).T

            for i in range(pixel_points.shape[0]):
                px, py = pixel_points[:,i]
                self.map[py, px] = 1

            self.plan_path()
            self.global_time = time.time()


    def map_cb(self, msg):
        pos = msg.info.origin.position # contains .x and .y
        quat = msg.info.origin.orientation
        eul = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        self.origin = np.array([pos.x, pos.y, pos.z, eul[2]])
        rospy.loginfo("ORIGIN: %s", self.origin)


        self.res = msg.info.resolution
        print('MAP RESOLUTION:', self.res)

        # msg.data is just a big array
        self.map_height = msg.info.height
        self.map_width = msg.info.width

        self.map = np.reshape(np.array(msg.data), (self.map_height, self.map_width))
        # self.map = np.reshape(np.array(msg.data), (self.map_height, self.map_width))
        self.map = ndimage.binary_dilation(self.map, structure=ndimage.generate_binary_structure(2, 2), iterations=7).astype(self.map.dtype)
        # print('MAP', self.map)


        rospy.loginfo("self.map shape: %s",self.map.shape)
        # if self.graph is None:
        #     self.convert_to_nodes()
        nice = self.pixel_to_actual((20,21))
        print('NICE', nice)
        cool = self.actual_to_pixel(nice)
        print('COOL', cool)



    # TODO(lori): convert to big dict, but how?
    # def convert_to_nodes(self):
    #     # creating dictionary with grid position and probability
    #     self.graph = {Node(i[0][0],i[0][1]):i[1] for i in np.ndenumerate(self.map) if i[1] == 0}
    #     print(self.graph)

    #TODO: treat above functions as black boxes, how to call them, and piece puzzle blocks together

    def odom_cb(self, msg):
        odom_world = PoseStamped()
        odom_world.header = msg.header
        odom_world.pose = msg.pose.pose

        transformed_pose = self.tf_listener.transformPose("/map", odom_world)


        #
        # marker = PoseStamped()
        # marker.header.frame_id = "/map"
        # marker.header.stamp = rospy.Time.now()
        #
        # marker.pose.orientation.x = msg.pose.orientation.x
        # marker.pose.orientation.y = msg.pose.orientation.y
        # marker.pose.orientation.z = msg.pose.orientation.z
        # marker.pose.orientation.w = msg.pose.orientation.w
        # marker.pose.position.x = msg.pose.position.x
        # marker.pose.position.y = msg.pose.position.y
        # marker.pose.position.z = 0.0
        #
        # self.ps_pub.publish(marker)
        #
        self.global_pose = transformed_pose.pose
        _start = transformed_pose.pose.position
        # _start = msg.pose.pose.position

        rospy.loginfo_once("START: %s, %s", _start.x, _start.y)

        self.global_start = (_start.x, _start.y)
        self.all_nodes.add(self.global_start)
        self.start = Node(*self.actual_to_pixel((_start.x, _start.y)))

        rospy.loginfo_once("START PIXEL: %s", self.start)

        if self.fake_goal is None:
            return

        # check if self.start is a pixel or actual
        if np.linalg.norm(np.array([self.global_pose.position.x - self.fake_goal[0], self.global_pose.position.y - self.fake_goal[1]])) < self.replan_distance:

            goal_line = np.array([self.global_goal[0] - self.global_start[0], self.global_goal[1] - self.global_start[1]])
            self.fake_goal = tuple(np.array([self.global_start[0], self.global_start[1]]) + (self.t)*goal_line)
            fake_node = node(*self.actual_to_pixel(self.fake_goal))

            increment = 0.05
            while self.map[fake_node.x, fake_node.y] != 0:

                self.fake_goal = np.array([self.global_start[0], self.global_start[1]]) + (self.t + increment)*goal_line
                fake_node = self.actual_to_pixel(self.fake_goal)
                increment += 0.05

            self.t += 0.1


            self.path_planning()

        # origin_node = PoseStamped()
        # origin_node.header = msg.header
        # origin_node.pose.position.x = self.origin[0]
        # origin_node.pose.position.y = self.origin[1]
        #
        # origin_node = Node(*self.actual_to_pixel((origin_node.pose.position.x, origin_node.pose.position.y)))
        # print("ORIGIN NODE:", origin_node)



    def goal_cb(self, msg):

        # odom_world = PoseStamped()
        # odom_world.header = msg.header
        # odom_world.pose = msg.pose
        #
        # msg = self.tf_listener.transformPose("/map", odom_world)


        _goal = msg.pose.position

        rospy.loginfo("GOAL: %s, %s", _goal.x, _goal.y)

        self.goal = Node(*self.actual_to_pixel((_goal.x, _goal.y)))
        self.global_goal = (_goal.x, _goal.y)
        rospy.loginfo("START PIXEL: %s", self.start)
        rospy.loginfo("GOAL PIXEL: %s", self.goal)
        rospy.loginfo("GOAL PIXEL CONVERTED BACK: %s", self.pixel_to_actual((self.goal.x, self.goal.y)))

        goal_line = np.array([self.global_goal[0] - self.global_start[0], self.global_goal[1] - self.global_start[1]])
        self.fake_goal = tuple(np.array([self.global_start[0], self.global_start[1]]) + (self.t)*goal_line)

        print('FAKE GOAL', self.fake_goal)


        if all(i is not None for i in [self.start, self.goal, self.map]):
            self.plan_path()


    def plan_path(self):
        ## CODE FOR PATH PLANNING ##

        # path = self.a_star_search()
        graph = self.rrt_star_search()
        print('GRAPH', graph)
        path = self.return_path(tuple(self.fake_goal))
        print('PATH AFTER RRT*:', path)
        # for node in path:
        #     x, y = self.pixel_to_actual((node.x, node.y))
        #     self.trajectory.addPoint(Point(x, y, 0))

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()
        print('BIG SUCCESS')


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
