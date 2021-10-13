import numpy as np
import rospy
import csv
import cv2
from cv_bridge import CvBridge, CvBridgeError

from tf import TransformListener

line_color = np.array([25,33,179],np.uint8)
lower,higher = np.array([15,17,175],np.uint8),np.array([45,35,185],np.uint8)
lower = np.array(lower,dtype= "uint8")
higher = np.array(higher,dtype = "uint8")
cv_image = cv2.imread('mod3test.png')
cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGBA2RGB)
cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)
# dim_x,dim_y,lol = cv_image.shape
#print(dim)

# cv2.imshow('image',cv_image)
#cv2.waitKey(0)
#cv2.destroyAllWindows()
#create and apply mask to include bottom 2/3 of image
# cv_image =cv2.cvtColor(cv_image,cv2.COLOR_RGBA2HSV)
lowerBound=np.array([3,149,99])
upperBound=np.array([16,255,251])

crop = np.zeros(cv_image.shape[:2], dtype="uint8")
cv2.rectangle(crop, (0, 200), (800, 800), 255, -1)
cropped = cv2.bitwise_and(cv_image, cv_image, mask=crop)
# cropped = cv2.inRange(cropped, line_color,line_color)
cropped = cv2.inRange(cropped, lowerBound, upperBound)
cv2.imshow("image", cropped)
cv2.waitKey(0)
cv2.destroyAllWindows()
edges = cv2.Canny(cropped, 50, 200)
# Detect points that form a line
lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=5, minLineLength=10, maxLineGap=250)
print('lines', lines)
m_list = []
b_list = []

x1_list = []
x2_list = []
y1_list = []
y2_list = []
# Draw lines on the image
for line in lines:
    x1, y1, x2, y2 = line[0]
    cv2.line(cv_image, (x1, y1), (x2, y2), (255, 0, 0), 3)
    m = (y2-y1)/(x2-x1)
    b = y1-m*x1
    # m_list.append(m)
    # b_list.append(b)
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

# m_av = sum(m_list)/len(m_list)
# b_av = sum(b_list)/len(b_list)
# cv2.line(cv_image, (1,1*m_av+b_av), (100, 100*m_av+b_av),(0,255,0),3)
# print('MB: ',(m_av,b_av))



# Show result
'''
cv2.imshow("Result Image", cv_image)
cv2.waitKey(0)
'''
# cv2.imshow("Mask Applied to Image", im2)
# cv2.imshow("Contours", im2)

#output = cv2.inRange(cv_image,line_color,line_color)
#output = cv2.bitwise_and(cv_image,cv_image,mask = mask)
#cv2.imshow("Mask Applied to Image", output)
#cv2.waitKey(0)
