import cv2 # OpenCV library
import numpy as np
import rospy
import time
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from sensor_msgs.msg import Image


def return_circle_or_square():
    # Creating nested list, where the:
                                    # first element specifies whether it is circle (0), or square (1)
                                    # second element specifies object centrepoint
                                    # third element specifies object width
                                    # fourth element specifies angle
    nested_list = []
    return_element_type = 0
    width = 500
    angle = 0

    wait_key_time=100
    # """
    rospy.loginfo("receiving video frame")
    time.sleep(5)
    msg = rospy.wait_for_message("camera/image_within_min_and_max_range", Image, timeout=30)
    br = CvBridge()
    original = br.imgmsg_to_cv2(msg)
    # """
    # original = cv2.imread("finding_shapes/output_image.jpg", cv2.IMREAD_COLOR)
    cv2.imshow('Original', original)
    cv2.waitKey(wait_key_time)
    # print("Reso: ", original.shape)

    # GREYSCALE IMAGE
    grey = cv2.cvtColor(original, cv2.COLOR_BGR2GRAY)
    cv2.imshow('Grayscale', grey)
    cv2.waitKey(wait_key_time)

    # GAUSSIAN BLUR IMAGE
    grey_gaussian_blur = cv2.GaussianBlur(grey,(7,7),cv2.BORDER_DEFAULT)
    cv2.imshow('Blurred & Greyscale', grey_gaussian_blur)
    cv2.waitKey(wait_key_time)

    # Apply thresholding to create a binary image
    _, binary_image = cv2.threshold(grey_gaussian_blur, 128, 255, cv2.THRESH_BINARY)
    cv2.imshow('Binary', binary_image)
    cv2.waitKey(wait_key_time)

    # Find contours in the binary image
    contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ###### Just for Displaying Contour Image ###############################
    contour_image = np.zeros_like(original)
    cv2.drawContours(contour_image, contours, -1, (0, 255, 0), 2)
    # Display the image with the drawn contours
    cv2.imshow('Contours', contour_image)
    cv2.waitKey(wait_key_time)
    ##################################################################

    image_1 = original.copy()  # Create a copy of the original image for image_1
    image_2 = original.copy()  # Create a copy of the original image for image_2

    circles = cv2.HoughCircles(grey_gaussian_blur, cv2.HOUGH_GRADIENT, 1, grey_gaussian_blur.shape[0] / 12, param1=100, param2=19, minRadius=50, maxRadius=150)
    print("Printing Circles: \n", circles)
    # Function to see the circles

    if circles is not None:
        return_element_type=0
        angle=0
        # Convert the (x, y) coordinates and radius of the circles to integers
        circles = np.uint16(np.around(circles))
        

        # Draw the circles on the output image
        for circle in circles[0, :]:
            center = (circle[0], circle[1])  # Circle center
            radius = circle[2]  # Circle radius
            cv2.circle(image_1, center, radius, (0, 0, 255), 2)  # Draw the circle in green with a thickness of 2 pixels
            cv2.circle(image_1, center, 5, (0, 0, 255), -1)  # Draw a small red circle at the center point
            nested_list.append([return_element_type, center, width, angle])

        # Show the image with overlaid circles
        cv2.imshow('Circles Detected', image_1)        # Function to see the circles
        cv2.waitKey(wait_key_time)


    image_2=original
    for contour in contours:
        # Approximate the contour to a polygon with fewer vertices
        epsilon = 0.04 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)


        if len(approx) == 4:
            # Calculate the orientation (angle) of the square
            rect = cv2.minAreaRect(approx)
            angle = rect[2]

            # Normalize the angle to be in the range [0, 180)
            if angle < -45:
                angle += 90

            # Calculate the center point of the square
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
            else:
                cX, cY = 0, 0

            # Print the center point coordinates and the orientation angle in the terminal
            print(f"Center Point: ({cX}, {cY}), Orientation Angle: {angle:.2f} degrees")

            # Calculate the aspect ratio of the bounding rectangle
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            # Check if the aspect ratio is close to 1 (square)
            if 0.9 < aspect_ratio < 1.1:
                # Draw a green rectangle around the square
                cv2.drawContours(image_2, [approx], -1, (0, 255, 0), 2)

                # Display the center point as a circle on the image
                center = (cX, cY)  # Circle center
                cv2.circle(image_2, center, 5, (0, 255, 0), -1)  # Draw a small red circle at the center point
                return_element_type=1
                nested_list.append([return_element_type, center, width, angle])

    # Display the image with squares and center points as circles
    cv2.imshow('Squares with Center Points', image_2)
    cv2.waitKey()
    cv2.destroyAllWindows()


    return nested_list


if __name__ == '__main__':
    try:
        rospy.init_node('temp')
        print(return_circle_or_square())

    except rospy.ROSInterruptException:
        pass