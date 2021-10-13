#!/usr/bin/env python2

import numpy as np
import rospy
import csv
import cv2
from visual_servoing_tesse.msg import LaneLine
from cv_bridge import CvBridge, CvBridgeError
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan, Image
from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from tf import TransformListener

class LineFinder:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("scan_topic", "/tesse/front_lidar/scan")
    SEG_CAM_TOPIC = rospy.get_param("seg_cam_topic", "/tesse/seg_cam/rgb/image_raw")

    LANE_LINE_TOPIC = rospy.get_param("lane_line_topic", "/lane_line")
    STEERING_RADIUS = 2*np.pi/9 #tesse car 2pi/9

    def __init__(self, lane_rgba=[0,0,0,0]):
        self.lane_rgba = np.array(lane_rgba,np.uint8)

        #Publishers
        # self._drive_publisher = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self._lane_line_publisher = rospy.Publisher(self.LANE_LINE_TOPIC, LaneLine, queue_size=10)

        #Subscribers
        rospy.Subscriber(self.SEG_CAM_TOPIC, Image, self._onSegCamDataReceived)

        #CV Bridge
        self.bridge = CvBridge() #Converts between ROS images and OpenCV Images

    def _onSegCamDataReceived(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        #create and apply mask to include bottom 1/2 of image
        # what was here before, i didn't want to delete it just in case
        # crop = np.zeros(cv_image.shape[:2], dtype="uint8")
        # cv2.rectangle(crop, (0, 200), (800, 800), 255, -1)
        # cropped = cv2.bitwise_and(cv_image, cv_image, mask=crop)
        # #isolate lane lines
        # new_image = np.zeros((cv_image.shape[0], cv_image.shape[1]))
        # mask = cv.inRange(cv_image, self.lane_rgba, self.lane_rgba)
        # new_img = cv.bitwise_and(cv_image, cv_image, mask = mask)
        # new_img = cv.cvtColor(new_img, cv.COLOR_RGB2GRAY)
        # cv2.imshow("linez", new_img)
        #Hough transform

        #line_color = np.array([25,33,179],np.uint8)
        #cv_image = cv2.imread('mod3test.png')
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)
        #cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

        #lowerBound=np.array([3,149,99])
        #upperBound=np.array([16,255,251])

        crop = np.zeros(cv_image.shape[:2], dtype="uint8")
        cv2.rectangle(crop, (0, 200), (800, 800), 255, -1)
        cropped = cv2.bitwise_and(cv_image, cv_image, mask=crop)

        mask = cv2.inRange(cropped, self.lane_rgba[:3], self.lane_rgba[:3])
        image_print(mask)

        #cropped = cv2.inRange(cropped, lowerBound, upperBound)
        #cropped = cv2.inRange(cropped, self.lane_rgba[:3], self.lane_rgba[:3])
        
        edges = cv2.Canny(mask, 50, 200)
        #rospy.loginfo(edges)
        # Detect points that form a line
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=5, minLineLength=10, maxLineGap=250)

        x1_list = []
        x2_list = []
        y1_list = []
        y2_list = []
        # Draw lines on the image
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 3)

            x1_list.append(x1)
            x2_list.append(x2)
            y1_list.append(y1)
            y2_list.append(y2)

        x1 = sum(x1_list)/len(x1_list)
        x2 = sum(x2_list)/len(x1_list)
        y1 = sum(y1_list)/len(x1_list)
        y2 = sum(y2_list)/len(y2_list)

        m = (y2-y1)/(x2-x1)
        b = y1-m*x1

        cv2.line(cv_image, (x1, y1), (x2, y2), (0, 0,255), 3)

        #publish average line
        lane_msg = LaneLine()
        lane_msg.m = m
        lane_msg.b = b
        print(m,b)
        self._lane_line_publisher.publish(lane_msg)


def image_print(img):
    """
    Helper function to print out images, for debugging. Pass them in as a list.
    Press any key to continue.
    """
    cv2.imshow("image", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def load_lane_color(path, semantic_label):
    """
    Helper function to extract color associated with semantic label
    """
    with open(path) as csvfile:
        reader = csv.DictReader(csvfile, fieldnames = ['label','r','g','b','a'])
        for row in reader:
            if row['label'] == semantic_label:
              return [int(row['r']), int(row['g']), int(row['b']), int(row['a'])]

if __name__ == "__main__":
    rospy.init_node('line_finder')
    #csv file and semantic label to extract color
    csv_path = rospy.get_param("/line_finder/csv_path")
    lane_semantic_label = "Decal_Road_Border_left(Clone)"
    color = load_lane_color(csv_path, lane_semantic_label)
    rospy.loginfo("lane color found: " + str(color))

    line_finder = LineFinder(lane_rgba = color)

    rospy.spin()
