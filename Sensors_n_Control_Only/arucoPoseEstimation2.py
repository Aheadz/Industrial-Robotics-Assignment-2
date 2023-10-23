import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ArucoPoseEstimator:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_pose_estimator', anonymous=True)

        # Create an ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters()

        # Derived camera parameters from Gazebo information
        w = 640
        h = 480
        theta = 1.089
        f = w / (2 * np.tan(theta / 2))
        self.camera_matrix = np.array([
            [f, 0, w/2],
            [0, f, h/2],
            [0, 0, 1]
        ])
        self.distortion_coeffs = np.array([0.0, 0.0, 0.0, 0.0])  # Assuming no distortion

        # Marker size in meters
        self.marker_size = 0.035  # 35mm converted to meters

        # Create a CvBridge to convert ROS image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback)
    def rotate_image(self, image, angle):
        # Get image dimensions
        (h, w) = image.shape[:2]
        # Define the pivot, which is the center of the image
        center = (w / 2, h / 2)

        # Perform the rotation
        M = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated = cv2.warpAffine(image, M, (w, h))
        return rotated

    def callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv_image = self.rotate_image(cv_image, -90)

        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.distortion_coeffs)
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            for i, corner in enumerate(corners):
                rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                se3_matrix = np.eye(4)
                se3_matrix[:3, :3] = rotation_matrix
                se3_matrix[:3, 3] = tvecs[i].flatten()
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
