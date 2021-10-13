#!/usr/bin/env python

import tf
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray, Point, PointStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from math import *
import sys
# np.set_printoptions(threshold=sys.maxsize)
import heapq
import matplotlib.pyplot as plt
from scipy import ndimage

#TODO (george): A* representation and algorithm, I NEED HELP THO

class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return not self.elements

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

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

        self.start = None
        self.goal = None
        self.map = None
        self.map_height = None
        self.map_width = None
        self.origin = None
        self.res = None
        # self.graph = None
        self.cost = dict()

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

    def a_star_search(self):
        # start and goal are Node objects, with start.cost = 0
        rospy.loginfo('Starting A* ...')

        def reconstruct_path(parent_dict):
            # follow parent pointers from goal to start, and reverse path and return
            path = [self.goal]
            while path[-1] != self.start:
                path.append(parent_dict[path[-1]])
            return list(reversed(path))

        frontier = PriorityQueue()
        parent = {self.start: None}
        frontier.put(self.start, 0)
        print("FINAL FRONTIER", frontier.elements)
        print("START/GOAL:", self.start, self.goal)
        time.sleep(10)
        self.cost[self.start] = 0
        while not frontier.empty():
            curr = frontier.get()
            # print("WHILE LOOP RUNNING:", curr)

            if curr == self.goal:
                rospy.loginfo('Goal! Reconstructing path...')
                return reconstruct_path(parent)

            for neighbor in self.get_neighbors(curr):
                # rospy.loginfo("NEIGHBOR:", neighbor)
                new_cost =  self.get_cost(curr, neighbor)
                # if neighbor.cost is None or new_cost < neighbor.cost:
                if neighbor not in self.cost or new_cost < self.cost[neighbor]:
                    # neighbor.cost = new_cost
                    self.cost[neighbor] = new_cost
                    priority = new_cost + self.get_heuristic(neighbor)
                    frontier.put(neighbor, priority)
                    parent[neighbor] = curr

        rospy.logerr("RETURNING NONE! DO NOT REACH HERE, WE DED BOIS")
        return None # we failed



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
        
        # Visualizing bitmap
        # plt.imshow(self.map)
        # plt.show()

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
        _start = transformed_pose.pose.position
        # _start = msg.pose.pose.position

        rospy.loginfo_once("START: %s, %s", _start.x, _start.y)

        self.start = Node(*self.actual_to_pixel((_start.x, _start.y)))
        rospy.loginfo_once("START PIXEL: %s", self.start)

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
        rospy.loginfo("START PIXEL: %s", self.start)
        rospy.loginfo("GOAL PIXEL: %s", self.goal)
        rospy.loginfo("GOAL PIXEL CONVERTED BACK: %s", self.pixel_to_actual((self.goal.x, self.goal.y)))


        if all(i is not None for i in [self.start, self.goal, self.map]):
            self.plan_path(self.start, self.goal, self.map)


    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##

        path = self.a_star_search()
        print('PATH AFTER A*:', path)
        for node in path:
            x, y = self.pixel_to_actual((node.x, node.y))
            self.trajectory.addPoint(Point(x, y, 0))

        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
