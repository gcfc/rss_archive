#!/usr/bin/env python2

import cv2 as cv
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from computer_vision.color_segmentation import cd_color_segmentation, image_print

class ConeDetector:
    #Semantic RGB of the cone object
    SEG_LABEL = "model"
    CONE_COLOR = np.asarray([194,253,94])
    SUB_TOPIC = "/tesse/seg_cam/rgb/image_raw"
    PUB_TOPIC = "/relative_cone_px"
    def __init__(self):
        self.sub = rospy.Subscriber(self.SUB_TOPIC, Image, self.callback)
        self.pub = rospy.Publisher(self.PUB_TOPIC, PointStamped, queue_size=5)

    def callback(self, msg):
        # Cone determined in subscribed message
        # doc here: http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # cv_image = cv.cvtColor(cv_image, cv.COLOR_RGB2BGR)
        bounding_box = self.semantic_segmentation(cv_image, None)
        top_left = bounding_box[0]
        bottom_right = bounding_box[1]
        midpoint = ((bottom_right[0] + top_left[0])/2.0, bottom_right[1])
        new_msg = PointStamped()
        new_msg.header.stamp = rospy.Time.now()
        new_msg.point.x = midpoint[0]
        new_msg.point.y = midpoint[1]
        self.pub.publish(new_msg)

    def semantic_segmentation(self, img, template):
    	"""
    	Implement the cone detection using color segmentation algorithm
    	Input:
    		img: np.3darray; the input image with a cone to be detected. BGR.
    		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
    	Return:
    		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
    				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
    	"""
    	########## YOUR CODE STARTS HERE ##########
        new_image = np.zeros((img.shape[0], img.shape[1]))

        mask = cv.inRange(img, self.CONE_COLOR, self.CONE_COLOR)

        new_img = cv.bitwise_and(img, img, mask = mask)

        new_img = cv.cvtColor(new_img, cv.COLOR_RGB2GRAY)


    	#visualize contours on cone
    	im2, contours, hierarchy = cv.findContours(new_img ,cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    	cv.drawContours(img, contours,-1,(0,255,0),3)

    	#initialize variables for bounding box
    	max_area = 0
    	rectangle = None
    	scale_min = 0.98
    	scale_max = 1.03

    	##create bounding box based on area of cone contour
    	for i in range(len(contours)):
    		area = cv.contourArea(contours[i])
    		if area > max_area:
    			max_area = area
    			rectangle = cv.boundingRect(cv.approxPolyDP(contours[i],3,True))

    	#determine x,y coords of bounding box using rectangle and multiplying by scaling factor
    	if rectangle != None:

    		xy_min = (int(scale_min*rectangle[0]),int(scale_min*rectangle[1]))
    		xy_max = (int(scale_max*(xy_min[0]+rectangle[2])),int(scale_max*(xy_min[1]+rectangle[3])))

    		#add box to image
    		cv.rectangle(img, xy_min, xy_max, (0,255,0),2)
    		image_print(img)

    		bounding_box = (xy_min,xy_max)
    	else:
    		bounding_box = ((0,0),(0,0))


    	########### YOUR CODE ENDS HERE ###########

    	# Return bounding box
    	return bounding_box


#LOTS OF COOL BUILTINS
# bitwise_and (mask) --> make everything a 0 or 1. only 1's where cones are.
# inrange
# take segment image message --> make into cv object
# cvt color to cvt.COLOR_RGB2GRAY
# apply simple algorithm to find contours super good at finding cone.
# if doesn't find, return something so we know.
# publishes relative cone pixels and send into homography matrix.
# send the middle of it... pretty good approximation.
# might want to drive to the base of the cone instead of the mass
# specific values from a csv.
# specific colors from various spots... cone is involved in csv and check what color is associated
# with the cone.

if __name__ == "__main__":
    rospy.init_node("cone_detector");
    node = ConeDetector()
    rospy.spin()
