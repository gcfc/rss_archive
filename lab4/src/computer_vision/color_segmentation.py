import cv2
import imutils
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(1)

	# cv2.waitKey(0)
	# cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
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

	#convert image rom bgr to hsv
	hsv_img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

	#lower and upper bounds for orange pixel range


	orange_lower = np.array([1, 225,140],np.uint8)
	orange_upper = np.array([28,255,255],np.uint8)

	#detect object (cone) based on range of pixel values in hsv colorspace
	new_img = cv2.inRange(hsv_img, orange_lower,orange_upper)

	# kernel = np.ones((2,2),np.uint8)
	# kernel2 = np.ones((1,1),np.uint8)
	# new_img = cv2.erode(new_img,kernel)
	# new_img = cv2.dilate(new_img,kernel2)

	# image_print(new_img)

	#visualize contours on cone
	im2, contours, hierarchy = cv2.findContours(new_img,cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	cv2.drawContours(img, contours,-1,(0,255,0),3)

	#initialize variables for bounding box
	max_area = 0
	rectangle = None
	scale_min = 0.98
	scale_max = 1.03

	##create bounding box based on area of cone contour
	for i in range(len(contours)):
		area = cv2.contourArea(contours[i])
		if area > max_area:
			max_area = area
			rectangle = cv2.boundingRect(cv2.approxPolyDP(contours[i],3,True))

	#determine x,y coords of bounding box using rectangle and multiplying by scaling factor
	if rectangle != None:

		xy_min = (int(scale_min*rectangle[0]),int(scale_min*rectangle[1]))
		xy_max = (int(scale_max*(xy_min[0]+rectangle[2])),int(scale_max*(xy_min[1]+rectangle[3])))

		#add box to image
		cv2.rectangle(img, xy_min, xy_max, (0,255,0),2)
		image_print(img)

		bounding_box = (xy_min,xy_max)
	else:
		bounding_box = ((0,0),(0,0))

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

if __name__ == "__main__":
	pass
