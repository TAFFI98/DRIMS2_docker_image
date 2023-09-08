#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
class Calibration:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/oak/rgb/image_raw', Image, self.callback)

    def callback(self, data):
        """
        Callback function for the image subscriber.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)                
        # Find the chess board corners               
        # If desired number of corners are found in the image then ret = true
        # Defining the dimensions of checkerboard
        CHECKERBOARD = (8,6)
        # Defining the size (in mm) of the checkboard squares
        square_size_mm = 18
        # Termination criteria
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        if ret:
                cv2.drawChessboardCorners(cv_image, CHECKERBOARD, corners, ret)
                print("Corners found!")

        object_points = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
        object_points[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
        object_points *= square_size_mm

        cv2.imshow('Chessboard', cv_image)
        cv2.waitKey(1)  # Display the image for a short time (ms)

        intrinsic_matrix = np.array([[1011.9113159179688, 0.0, 655.5979614257812], [0.0, 1011.9113159179688, 372.8844909667969],  [0.0, 0.0, 1.0]])
        distortion_coefficients = np.array([38.68403244018555, -284.3472900390625, -0.0023334568832069635, 0.0014240274904295802, 617.3338623046875, 38.054351806640625, -280.6847839355469, 609.9551391601562])
        _, rotation_vector, translation_vector, inliers = cv2.solvePnPRansac(object_points, corners, intrinsic_matrix, distortion_coefficients)
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        print(translation_vector)

        



if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('image_processor', anonymous=True)
    processor = Calibration()
    rospy.spin()
    


