#!/usr/bin/env python3
import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
import tf
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# ==========================
#  Dependencies:
#  - opencv-contrib-python                4.5.5.62
#  - opencv-python                        4.8.0.76
#  - numpy                                1.24.4
#  - rospy                                1.16.0
# ==========================

class ArucoPoseEstimator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_pose_estimator', anonymous=True)
        
        # Load camera calibration parameters from .yml file
        #calibration_path = rospy.get_param('~calibration_file', 'calibrated.yml')
        #fs = cv2.FileStorage(calibration_path, cv2.FILE_STORAGE_READ)
        
        # Read the camera matrix
        #self.camera_matrix = np.array(fs.getNode("camera_matrix").mat())
        
        # Read the distortion coefficients
        #self.distortion_coeffs = np.array(fs.getNode("distortion_coefficients").mat())
        #.distortion_coeffs = self.distortion_coeffs.reshape(-1)  # Flatten to 1D array
        
        # Check if using a simulated camera
        self.simulated_camera = rospy.get_param('~simulated_camera', True)
        
        # Create an ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()
        
        #Simulation Camera has same intrinsic parameters as Real camera
        w = 640
        h = 480  
        fx = 596.66585835
        fy = 591.17132116
        cx = 279.03751085
        cy = 197.77605461
        self.distortion_coeffs = np.array([
            -0.23880386371963863, -0.7214320219570768, 
            -0.00037858100167365054, 0.0025032179171072936,1.941908386385502])   
        
        self.camera_matrix = np.array([
            [fx, 0,  cx],
            [0,  fy, cy],
            [0,  0,   1]
        ])
        
        # Marker size in meters
        self.marker_size = 0.033  # 35mm converted to meters
        # Create a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()
        # Subscribe to the camera image topic
        if (self.simulated_camera):
            self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
        else:
            self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.pose_pub = rospy.Publisher("/aruco_tags/pose",PoseStamped, queue_size=1000)
        self.sequenceNum = 0
        
    def rotate_image(self, image, angle):
        # Get image dimensions
        (h, w) = image.shape[:2]
        # Define the pivot, which is the center of the image
        center = (w / 2, h / 2)

        # Perform the rotation
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv2.warpAffine(image, M, (w, h))
        return rotated
    # Compute Euler angles from rotation matrix
    def rotationMatrixToEulerAngles(self,R):
        sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
        singular = sy < 1e-6

        if not singular:
            x = np.arctan2(R[2, 1], R[2, 2])
            y = np.arctan2(-R[2, 0], sy)
            z = np.arctan2(R[1, 0], R[0, 0])
        else:
            x = np.arctan2(-R[1, 2], R[1, 1])
            y = np.arctan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def rotationMatrixToQuaternion(self,R):
        quaternion = tf.transformations.quaternion_from_matrix(R)
        return quaternion

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        if self.simulated_camera:
            cv_image = self.rotate_image(cv_image, -90)
            
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        se3_matrix = np.eye(4)
        rotation_matrix = np.eye(3)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.distortion_coeffs)
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i, corner in enumerate(corners):
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                se3_matrix = np.eye(4)
                se3_matrix[:3, :3] = rotation_matrix
                roll, pitch, yaw = self.rotationMatrixToEulerAngles(rotation_matrix)
                tveclist = tvecs[i][0].tolist()
                poseMsg = PoseStamped()
                poseMsg.header.seq = self.sequenceNum
                poseMsg.header.stamp = rospy.Time(0)
                poseMsg.header.frame_id = str(i)
                poseMsg.pose.position.x = tveclist[0]
                poseMsg.pose.position.y = tveclist[1]
                poseMsg.pose.position.z = tveclist[2]
                quaternion = self.rotationMatrixToQuaternion(se3_matrix.tolist())
                poseMsg.pose.orientation.x = quaternion[0]
                poseMsg.pose.orientation.y = quaternion[1]
                poseMsg.pose.orientation.z = quaternion[2]
                poseMsg.pose.orientation.w = quaternion[3]
                self.pose_pub.publish(poseMsg)
                self.sequenceNum+=1
                #print("x: %f ,y: %f , z: %f "  % (tveclist[0] ,tveclist[1],tveclist[2]))
                rpy = [roll,pitch,yaw]
                #print("Roll: %f , Pitch: %f , Yaw: %f " % (rpy[0]* 180/3.141596,rpy[1]* 180/3.141596,rpy[2]* 180/3.141596))
            for i in range(len(ids)):
                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.distortion_coeffs, rvecs[i], tvecs[i], 0.0175)
        
                
        cv2.imshow('Aruco Pose Estimation', cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    aruco_estimator = ArucoPoseEstimator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
