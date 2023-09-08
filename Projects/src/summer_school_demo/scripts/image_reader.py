#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
import tf.transformations as tf
import numpy as np
from geometry_msgs.msg import PoseStamped
class ImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/oak/rgb/image_raw', Image, self.callback)
        self.image_processed = rospy.Publisher('/oak/rgb/image_processed', Image, queue_size=10)
        self.face_number = rospy.Publisher('/vision_pub/dice_number', Int16 , queue_size=10)
        self.dice_pose = rospy.Publisher('/vision_pub/dice_pose', PoseStamped , queue_size=10)
        # DEFINE TRANSFORMATION MATRICES
        self.transf_from_camera_to_marker = np.linalg.inv(np.array([[1.0, 0.0, 0.0, 0.20317620967],
                                            [ 0.0, 1.0, 0.0, -0.2120121097],
                                            [0.0, 0.0,1.0, 0.73021775171],
                                            [0,0,0,1]])) # postion of marker in camera frame
        self.from_marker_to_robot = np.array([[-0.999957345392659, -0.007971200174093, 0.003975433282975, -0.141450000000000],
                                            [ -0.008320583227092, 0.999959975237488, 0.004094776806375, 0.129100000000000],
                                            [-0.004009400207873, 0.004062990379999, -0.999983647371856, 0.087900000000000],
                                            [0,0,0,1]]) # postion of marker in robot frame
        self.transf_from_camera_to_robot = self.from_marker_to_robot @ self.transf_from_camera_to_marker 


    def rotation_matrix_to_euler_angles(self,R):
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.degrees(x), np.degrees(y), np.degrees(z)

    def number_detector_and_publisher(self, mask, box, cv_image):

        # Find the minimum and maximum x and y coordinates of the box
        min_x = min(box[:, 0])
        max_x = max(box[:, 0])
        min_y = min(box[:, 1])
        max_y = max(box[:, 1])
        # Set up the blob detector parameters
        params = cv2.SimpleBlobDetector_Params()
        # Change parameters as needed
        params.minThreshold = 0    # Minimum threshold value to detect blobs
        params.maxThreshold = 20   # Maximum threshold value to detect blobs
        params.filterByArea = True  # Filter by area (set to True to filter)
        params.minArea = 8        # Minimum area of the blob
        params.filterByCircularity = True  # Filter by circularity (set to True to filter)
        params.minCircularity = 0.75     # Minimum circularity of the blob (0-1)
        params.filterByConvexity = False  # Filter by convexity (set to True to filter)
        params.filterByInertia = False   # Filter by inertia (set to True to filter)
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs in the image
        keypoints = detector.detect(mask[min_y:max_y, min_x:max_x])
        mask_crop = cv_image[min_y:max_y, min_x:max_x]
        keypoints_img =  cv2.drawKeypoints(mask_crop, keypoints, None, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.namedWindow('Mask crop', cv2.WINDOW_NORMAL)
        #cv2.imshow("Mask crop",keypoints_img)
        #cv2.waitKey(1)
        # Print the number of detected blobs
        print(f"Number of blobs detected: {len(keypoints)}")
        number = len(keypoints)
        self.face_number.publish(number)

    def extract_mask(self, cv_image):

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        # Extract yellow pixels
        lower = np.array([20, 100, 100])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        return mask

    def extract_coordiantes_in_camera_frame(self, center_x, center_y):

        IntrinsicMatrix = np.array([[1011.9113159179688, 0.0, 655.5979614257812], [0.0, 1011.9113159179688, 372.8844909667969],  [0.0, 0.0, 1.0]])
        cx = 655.5979614257812
        cy = 372.8844909667969
        f = 1011.9113159179688
        z_camera = 0.73
        # Position in camera ref frame
        x_dice = (center_x -cx)* z_camera /f
        y_dice = (center_y -cy)* z_camera /f
        z_dice = 0.73 - 0.003
        camera_coordinates = np.array([x_dice, y_dice, z_dice])
        return camera_coordinates
    
    def pose_in_robot_frame(self, camera_coordinates, center_x, center_y, cv_image):
        camera_coordinates = np.array([camera_coordinates[0], camera_coordinates[1], camera_coordinates[2], 1 ])
        robot_coordinates = self.transf_from_camera_to_robot @ camera_coordinates

        # Create a string with the point's position
        point_text = f'({round(robot_coordinates[0], 5)}, {round(robot_coordinates[1], 5)})'

        # Define the font properties
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_color = (255, 255, 255)  # White color
        font_thickness = 1

        # Draw the text on the image
        cv2.putText(cv_image, point_text, (center_x, center_y), font, font_scale, font_color, font_thickness)

        print('Position of dice in robot coordinates frame: ', robot_coordinates[0:3])
        return robot_coordinates

    def pubish_dice_pose(self, robot_coordinates, quaternion):

        # Publish Dice pose
        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = rospy.Time.now()
        # Set the pose data (position and orientation)
        pose_stamped_msg.pose.position.x = robot_coordinates[0] 
        pose_stamped_msg.pose.position.y = robot_coordinates[1]   
        pose_stamped_msg.pose.position.z = robot_coordinates[2]   
        pose_stamped_msg.pose.orientation.x = quaternion[0]  # Replace with your desired orientation x
        pose_stamped_msg.pose.orientation.y = quaternion[1]  # Replace with your desired orientation y
        pose_stamped_msg.pose.orientation.z = quaternion[2]  # Replace with your desired orientation z
        pose_stamped_msg.pose.orientation.w = quaternion[3]  # Replace with your desired orientation w
        
        self.dice_pose.publish(pose_stamped_msg)

    def orientation_in_camera_frame(self, rotated_rect, center_x, center_y, cv_image ):

        #   ORIENTATION: Extract orientaiton of the box
        _, _, angle_degrees = rotated_rect
        angle_degrees = angle_degrees 
        
        # Calculate the rotation matrix in checkboard reference frame
        angle_radians = np.radians(angle_degrees)
        # Your 2D rotation matrix
        rotation_matrix_3d = np.array([[np.cos(angle_radians), -np.sin(angle_radians), 0],
                                        [np.sin(angle_radians), np.cos(angle_radians), 0],
                                        [0, 0, 1]])

        # Define the length of the coordinate axes
        axis_length = 60

        # Calculate the endpoints of the axes
        x_axis_endpoint = (int(center_x + axis_length * np.cos(angle_radians)), int(center_y - axis_length * np.sin(angle_radians)))
        y_axis_endpoint = (int(center_x - axis_length * np.sin(angle_radians)), int(center_y - axis_length * np.cos(angle_radians)))

        # Draw the coordinate axes
        cv2.line(cv_image, (center_x, center_y), x_axis_endpoint, (0, 0, 255), 2)  # Red X-axis
        cv2.line(cv_image, (center_x, center_y), y_axis_endpoint, (0, 255, 0), 2)  # Green Y-axis
        
        return angle_radians, rotation_matrix_3d
    
    def extract_orientation_in_robot_frame(self, rotation_matrix_checkboard):
        self.rotation_matrix_from_robot_to_checkboard = np.array([[-0.999957345392659, -0.007971200174093, 0.003975433282975],
        [ -0.008320583227092, 0.999959975237488, 0.004094776806375],
        [-0.004009400207873, 0.004062990379999, -0.999983647371856]])
        rotation_matrix_robot = np.dot(self.rotation_matrix_from_robot_to_checkboard, rotation_matrix_checkboard)
        roll, pitch, yaw = self.rotation_matrix_to_euler_angles(rotation_matrix_robot)
        return roll, pitch, yaw 
    
    def callback(self, data):
        """
        Callback function for the image subscriber.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        image_height, image_width, _ = cv_image.shape
        # Extract binary mask with yellow pixels
        mask  = self.extract_mask(cv_image)
        
        # Extract contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Assuming you want to find the oriented rectangle for the largest contour
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Calculate the oriented bounding rectangle
        rotated_rect = cv2.minAreaRect(largest_contour)
        
        # Draw the oriented bounding rectangle on a copy of the original image
        box = cv2.boxPoints(rotated_rect)
        box = np.int0(box)
        cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
        
        # Find the minimum and maximum x and y coordinates of the box
        min_x, max_x, min_y, max_y = min(box[:, 0]), max(box[:, 0]), min(box[:, 1]), max(box[:, 1])

        # Extract number on the dice
        self.number_detector_and_publisher(mask, box,cv_image)

        # POSITION: Extract the center point and convert it to integer (pixel coordinates)
        center_x, center_y = map(int, rotated_rect[0])

        # POSITION: Extract coordinates in camera ref frame
        camera_coordinates = self.extract_coordiantes_in_camera_frame(center_x,center_y)
        # ORIENTATION: Extract orientation of the dice in camera reference frame
        angle_radians, rotation_matrix_camera = self.orientation_in_camera_frame(rotated_rect, center_x, center_y, cv_image )

        # POSITION: Extract position of dice in robot frame
        robot_coordinates = self.pose_in_robot_frame(camera_coordinates, center_x, center_y, cv_image)

        # ORIENTATION: Extract orientaion of the dice in robot reference frame
        roll, pitch, yaw = self.extract_orientation_in_robot_frame(rotation_matrix_camera)
        quaternion = tf.quaternion_from_euler(roll, pitch, yaw)

        # Show image
        cv2.namedWindow('Dice', cv2.WINDOW_NORMAL)
        cv2.imshow("Dice",cv_image)
        cv2.waitKey(1)
        # Show mask
        # cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
        # cv2.imshow("Dice",mask)
        # cv2.waitKey(1)
        
        try:
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # Publish Dice pose
        self.pubish_dice_pose(robot_coordinates,quaternion)
        
        # Publish image
        self.image_processed.publish(ros_image)


if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('image_processor', anonymous=True)
    processor = ImageProcessor()
    rospy.spin()


