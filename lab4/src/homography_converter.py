#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped
import tf
from tf.transformations import euler_from_quaternion
import numpy as np
import cv2
from visual_servoing_tesse.msg import cone_location, LaneLine
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo


class HomographyConverter():
    """
    Rosnode for transforming pixel-frame topics into world-frame cone poses.

    Subscribes to a segmented line in the camera frame (lane_line_topic: LaneLine)
    and publishes a dummy cone position at self.LOOKAHEAD_DISTANCE ahead (for line
    following) to /relative_cone.

    Subscribes to a point in the camera frame (/relative_cone_px: PointStamped) and
    publishes the same point tranformed into the world frame to /relative_cone.
    """
    def __init__(self):

        # Subscribe to camera transform info
        #SEG_CAM_INFO_TOPIC = rospy.get_param("seg_cam_info_topic")
        SEG_CAM_INFO_TOPIC = "/tesse/seg_cam/camera_info"
        rospy.Subscriber(SEG_CAM_INFO_TOPIC,
            CameraInfo, self.seg_cam_info_callback)

        self.seg_intrinsic_matrix = None
        self.seg_extrinsic_matrix = None
        self.homography_matrix = None

        # Subscribe to clicked point messages from rviz
        #RELATIVE_CONE_PX_TOPIC = rospy.get_param("relative_cone_px_topic")
        #LANE_LINE_TOPIC = rospy.get_param("lane_line_topic")
        LANE_LINE_TOPIC = "/lane_line"

        rospy.Subscriber("/relative_cone_px",
            PointStamped, self.point_callback)
        rospy.Subscriber(LANE_LINE_TOPIC,
            LaneLine, self.line_callback)
        self.message_x = None
        self.message_y = None
        self.message_frame = "map"


        # lookahead distance (tunable control parameter)
        self.LOOKAHEAD_DISTANCE = 8.0


        self.cone_pub = rospy.Publisher("/relative_cone",
            cone_location,queue_size=1)
        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10) #10 hz

        while not rospy.is_shutdown():
            self.publish_cone()
            self.rate.sleep()

    def seg_cam_info_callback(self, msg):

        # raise NotImplementedError

        ############################################################################
        ### YOUR CODE HERE

        ## get the intrinsic parameters of the camera from ROS

        self.seg_intrinsic_matrix = np.array(msg.K).reshape((3, 3))

        ## manually fill in the extrinsic camera parameters using info in
        ## the lab handout. It helps to plot the axes of base_link_gt and
        ## left_cam in rviz, then think about how to construct the transform
        ## from of base_link_gt with respect to the camera.

        self.seg_extrinsic_matrix = np.array([[0,-1,0,0.05],
                                              [0,0,-1,1.03],
                                              [1,0,0,-1.5]])

        ## pick some points (at least four) in the ground plane
        PTS_GROUND_PLANE = np.array([[2.5, 1.0, 0.0, 1],
                                    [2.5, -1.0, 0.0, 1],
                                    [3.5, 1.0, 0.0, 1],
                                    [3.5, -1.0, 0.0, 1],])

        ## project those points to the image plane using the intrinsic and
        ## extrinsic camera matrices

        # transpose since each point is a column in formula, print out for details / debugging
        PTS_IMAGE_PLANE = self.seg_intrinsic_matrix.dot(self.seg_extrinsic_matrix).dot(PTS_GROUND_PLANE.T).T

        # normalize s.t. last col is 1's
        for i in range(len(PTS_IMAGE_PLANE)):
            PTS_IMAGE_PLANE[i,:] /= PTS_IMAGE_PLANE[i,-1]

        # relevant columns, both 4x2 matrices
        relevant_ground = np.float32(PTS_GROUND_PLANE[:,:2]) #nonzero columns
        relevant_image = np.float32(PTS_IMAGE_PLANE[:,:2]) # u,v columns, not the 1's

        ## finally, compute the homography matrix to backproject from image
        ## plane to ground plane

        # documentation here! https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga4abc2ece9fab9398f2e560d53c8c9780
        self.homography_matrix, err = cv2.findHomography(relevant_image, relevant_ground)

        #############################################################################


    def publish_cone(self):
        """
        Publish the relative location of the cone
        """
        # Find out most recent relative location of cone
        if self.message_x is None:
            return
        try:
            msg_frame_pos, msg_frame_quat = self.tf_listener.lookupTransform(
                "base_link_gt", self.message_frame, rospy.Time(0))
        except:
            return
        # Using relative transformations, convert cone in whatever frame rviz
        # was in to cone in base link (which is the control frame)
        (roll, pitch, yaw) = euler_from_quaternion(msg_frame_quat)
        cone_relative_baselink_x =\
            msg_frame_pos[0]+np.cos(yaw)*self.message_x-np.sin(yaw)*self.message_y
        cone_relative_baselink_y =\
            msg_frame_pos[1]+np.cos(yaw)*self.message_y+np.sin(yaw)*self.message_x

        # Publish relative cone location
        relative_cone = cone_location()
        relative_cone.x_pos = cone_relative_baselink_x
        relative_cone.y_pos = cone_relative_baselink_y
        self.cone_pub.publish(relative_cone)

    def draw_marker(self):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 1.2
        marker.scale.y = 1.2
        marker.scale.z = 1.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y
        self.marker_pub.publish(marker)


    def point_callback(self, msg):
        # get pixel coordinates
        px = msg.point.x
        py = msg.point.y

        # apply homography matrix
        if self.seg_intrinsic_matrix is None or self.seg_extrinsic_matrix is None:
            return

        '''
        # (hardcoded for now)
        H = np.array([[ 2.31630946e-05, -3.50827978e-05, -3.72600692e-01],
                      [ 1.09022607e-03, -2.89622858e-04, -3.13292098e-01],
                      [ 8.58918330e-05, -5.64162792e-03,  1.00000000e+00]])
        rospy.loginfo(H)
        '''

        H = self.homography_matrix

        CAM_PT = np.array([px, py, 1]).T

        WORLD_PT_UNSCALED = np.matmul(H, CAM_PT)
        WORLD_PT = WORLD_PT_UNSCALED / WORLD_PT_UNSCALED[-1]

        x, y = WORLD_PT[0], WORLD_PT[1] # coordinates in world frame

        # switch to rviz coordinate system
        self.message_x = x
        self.message_y = y
        self.message_frame = "base_link_gt"

        # Draw a marker for visualization
        self.draw_marker()


    def line_callback(self, msg):
        # get line parameters
        pm = msg.m
        pb = msg.b

        # get two points on the line (pixel coordinates)
        px0, py0 = 200, pm*200+pb
        px1, py1 = 300, pm*300+pb

        # apply homography matrix

        if self.seg_intrinsic_matrix is None or self.seg_extrinsic_matrix is None:
            return

        '''
        # (hardcoded for now)
        H = np.array([[ 2.31630946e-05, -3.50827978e-05, -3.72600692e-01],
                      [ 1.09022607e-03, -2.89622858e-04, -3.13292098e-01],
                      [ 8.58918330e-05, -5.64162792e-03,  1.00000000e+00]])
        rospy.loginfo(H)
        '''

        H = self.homography_matrix

        CAM_PT_0 = np.array([px0, py0, 1]).T
        CAM_PT_1 = np.array([px1, py1, 1]).T

        WORLD_PT_UNSCALED_0 = np.matmul(H, CAM_PT_0)
        WORLD_PT_0 = WORLD_PT_UNSCALED_0 / WORLD_PT_UNSCALED_0[-1]

        WORLD_PT_UNSCALED_1 = np.matmul(H, CAM_PT_1)
        WORLD_PT_1 = WORLD_PT_UNSCALED_1 / WORLD_PT_UNSCALED_1[-1]

        x0, y0 = WORLD_PT_0[0], WORLD_PT_0[1] # coordinates in world frame
        x1, y1 = WORLD_PT_1[0], WORLD_PT_1[1] # coordinates in world frame

        # find the parameterized line in the world frame using our two tranformed points
        m = (y1 - y0) / (x1 - x0)
        b = y0 - m * x0

        # find the point on the line that is LOOKAHEAD_DISTANCE ahead of the robot
        xlook = self.LOOKAHEAD_DISTANCE
        ylook = m * xlook + b

        #rospy.loginfo(("xlook", xlook))

        # switch to rviz coordinate system
        self.message_x = xlook
        self.message_y = ylook
        self.message_frame = "base_link_gt"

        # Draw a marker for visualization
        self.draw_marker()

if __name__ == '__main__':
    try:
        rospy.init_node('HomographyConverter', anonymous=True)
        HomographyConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
