import numpy as np
import cv2

## get the intrinsic parameters of the camera from ROS

seg_intrinsic_matrix = np.array(
    [190.68057481507364, 0.0, 240.0, 0.0, 190.6805748150736, 160.0, 0.0, 0.0, 1.0]
    ).reshape((3, 3))

## manually fill in the extrinsic camera parameters using info in
## the lab handout. It helps to plot the axes of base_link_gt and
## left_cam in rviz, then think about how to construct the transform
## from of base_link_gt with respect to the camera.

seg_extrinsic_matrix = np.array([[0,-1,0,0.05],
                                [0,0,-1,1.03],
                                [1,0,0,-1.5]])

## pick some points (at least four) in the ground plane
PTS_GROUND_PLANE = np.array([[2.5, 1.0, 0.0, 1],
                            [2.5, -1.0, 0.0, 1],
                            [3.5, 1.0, 0.0, 1],
                            [3.5, -1.0, 0.0, 1],])

print('GROUND truth (haha get it?)')
print(PTS_GROUND_PLANE)


## project those points to the image plane using the intrinsic and
## extrinsic camera matrices

# transpose since each point is a column in formula, print out for details / debugging
PTS_IMAGE_PLANE = seg_intrinsic_matrix.dot(seg_extrinsic_matrix).dot(PTS_GROUND_PLANE.T).T

# normalize s.t. last col is 1's
for i in range(len(PTS_IMAGE_PLANE)):
    PTS_IMAGE_PLANE[i,:] /= PTS_IMAGE_PLANE[i,-1]

# relevant columns, both 4x2 matrices
relevant_ground = np.float32(PTS_GROUND_PLANE[:,:2]) #nonzero columns
relevant_image = np.float32(PTS_IMAGE_PLANE[:,:2]) # u,v columns, not the 1's

## finally, compute the homography matrix to backproject from image
## plane to ground plane

# documentation here! https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga4abc2ece9fab9398f2e560d53c8c9780
homography_matrix, err = cv2.findHomography(relevant_image, relevant_ground)
print("\nhomography matrix: (last entry must be 1)")
print(homography_matrix)

res = np.array(homography_matrix).dot(PTS_IMAGE_PLANE.T).T

for i in range(len(res)):
    res[i,:] /= res[i,-1]
print("\nreconstructed result: ")
print(res)