# ! /usr/bin/python

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray

import math
import numpy as np
import time

from UR3e import *
from RG2 import *
from detecting_circle_or_square_function import *



if __name__ == '__main__':
    try:

        gripper = RG2()
        robot = UR3e()
        rospy.init_node('gripperAndRobotPublisher', log_level=rospy.INFO)

        msg = Image
        wait_key_time = 250
        image_width = 1280

        x_offset = 0.033
        y_offset = 0

        c_y_offset = 0
        s_y_offset = 0


        # GO TO STARTING LOCATION
        gripper.moveGripper(1100) # Open Gripper so to maximum that the camera can see alot more easier
        robot.setEndEffectorWithOrientation(0, 0.23, 0.45, math.pi, 0, 0)
        print("\n End Effector at Looking Down:")
        robot.jointsAndEndEffectorInformation()


        # POSE OF CAMERA
        pose = robot.group.get_current_pose(end_effector_link="wrist_3_link").pose
        print("\nPose of Wrist 3 Link:", pose)
        cameraPose = rotate_pose_msg_by_euler_angles(pose, 0, 0, 3.14)
        cameraPose = translate_pose_msg(cameraPose, 0, 0.045, 0.093)
        print("\nPose of Camera:", cameraPose)
        

        # GET IMAGE MESSAGES
        print("Waiting for 2 seconds so that image is stable")
        time.sleep(2)
        nested_list = return_circle_or_square() # FUNCTION TO MAKE A NESTED LIST(2D) OF EACH SHAPE
        print(nested_list)
        msg2 = rospy.wait_for_message("camera/XYZ_location_of_pixels", PoseArray, timeout=20) # Get the X,Y,Z position of every pixel on screen
        # print(len(msg2.poses)) # Printing the number of poses, which is the same number as number of pixels


        # GO TO EACH SHAPE
        for n in range(len(nested_list)):

            # INITIAL
            center = nested_list[n][1]  # Access the x and y coordinates of the center

            shape = nested_list[n][0]   # Determine if it's a circle or a square so I know home position
            if shape == 0:              # If shape is a circle
                direction = -1
            else:                       # If shape is a square
                direction = 1


            # GET ARRAY LOCATION AND POSE OF EACH PIXEL
            i = (center[1]*image_width)+center[0]
            print("Pixel Row: ", center[1])
            print("Pixel Column: ", center[0])
            print("Pixel Location: ", i)
            pose = msg2.poses[i]
            x = pose.position.x
            y = pose.position.y
            z = pose.position.z
            print("X position:", x, "Y position:", y, "Z position:", z)


            # SET YAW - MIGHT HAVE/NOT HAVE TO BE NEGATIVE????
            yaw = nested_list[n][3] * (pi/180) * -1
            print("\nYAW: ", yaw)

            # GO ABOVE THE BOTTLE
            bottle_above = translate_pose_msg(cameraPose, x, y, z-0.02)
            print("\nPose of item (but 2cm above):", bottle_above)
            robot.setEndEffectorOfFingerWithOrientation(gripper.depth, bottle_above.position.x + x_offset, bottle_above.position.y, bottle_above.position.z, math.pi, 0, yaw)


            # GO BELOW THE BOTTLE
            bottle_below = translate_pose_msg(cameraPose, x, y, z+0.01)
            print("\nPose of item (but 2cm below):", bottle_below)
            robot.setEndEffectorOfFingerWithOrientation(gripper.depth, bottle_below.position.x + x_offset, bottle_below.position.y, bottle_below.position.z, math.pi, 0, yaw)
            gripper.moveGripper(500)  # Close Gripper


            # BRING BOTTLE UP
            robot.setEndEffectorOfFingerWithOrientation(gripper.depth, bottle_above.position.x + x_offset, bottle_above.position.y, bottle_above.position.z+0.035, math.pi, 0, yaw)


            # GO TO BIN LOCATION
            print("Going to bin location")
            robot.offsetSpecificJoint(0, direction*math.pi/2*0.75) # Joint Waypoint so that it is at the approximate drop-off location

            if shape == 0: # Everytime an item is placed on the table, it is important to ensure that it goes in a different position without colliding
                bin_y_offset = c_y_offset
                c_y_offset += 0.075
            else:
                bin_y_offset = s_y_offset
                s_y_offset += 0.075

            robot.setEndEffectorOfFingerWithOrientation(gripper.depth, -1*direction*0.35, bin_y_offset, 0.05, math.pi, 0, 0) # Drop the item at a specific location, such that it does not interfere with other existing items
            

            # AFTER DROPPING GO BACK TO HOME LOCATION
            gripper.moveGripper(1100)                                           # Open Gripper
            robot.offsetSpecificJoint(1, -1/3)                                  # Lift Shoulder Joint so it doesn't colide with other items on print bed
            robot.changeSpecificJoint(0, 0.9645)                                # Move Base Joint back to 'LOOKING DOWN' position
            robot.setEndEffectorWithOrientation(0, 0.23, 0.45, math.pi, 0, 0)   # Go to 'LOOKING DOWN' position



    except rospy.ROSInterruptException:
        pass