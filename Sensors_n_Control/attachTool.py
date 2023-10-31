import rospy
import moveit_commander
import sys
import numpy as np
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
import numpy as np
from tf.transformations import *
from sensor_msgs.msg import JointState
import actionlib
from moveit_msgs.msg import MoveGroupAction

import math

class RobotController:
    def __init__(self, group_name='tmr_arm'):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('visual_servoing_program', anonymous=True)
        
        self.client = actionlib.SimpleActionClient('move_group', MoveGroupAction)
        self.client.wait_for_server()
        # PID gains (you might need to tune these values)
        self.kp = [1.0, 1.0, 1.0]  # Proportional gains for x, y, z
        self.ki = [0.0, 0.0, 0.0]  # Integral gains for x, y, z
        self.kd = [0.1, 0.1, 0.1]  # Derivative gains for x, y, z

        # Error accumulators for integral term
        self.error_integral = [0.0, 0.0, 0.0]

        # Previous error for derivative term
        self.prev_error = [0.0, 0.0, 0.0]

        # Create a MoveGroupCommander object
        self.group = moveit_commander.MoveGroupCommander(group_name)
        
        self.aruco_poses = {}
        # Member variable to store unique IDs
        self.aruco_ids = []
        # Member variable to store joint states corresponding to unique IDs These are those where the robot can see the tags from.
        self.joint_states_for_ids = {}
        # Subscribe to the topic
        self.aruco_subscriber = rospy.Subscriber("/aruco_tags/pose", PoseStamped, self.aruco_callback)
        self.process_aruco_data = False
        # Member variable to store printerID
        self.printerID = None
        # Subscribe to the topic
        self.printer_subscriber = rospy.Subscriber("/gotoPrinter", Int8, self.printer_callback)
        
        # Member variable to store the latest joint values
        self.latest_joint_values = []
        
        # Subscribe to the joint_states topic to get the current joint values
        self.joint_state_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        
        # Subscribe to the /desired_joint_state topic
        self.desired_joint_state_subscriber = rospy.Subscriber("/desired_joint_state", JointState, self.desired_joint_state_callback)
   
    def is_trajectory_running(self):
        state = self.client.get_state()
        print(state)
        return state == actionlib.GoalStatus.PENDING or state == actionlib.GoalStatus.ACTIVE

    def wait_for_trajectory_completion(self):
        while self.is_trajectory_running():
            rospy.sleep(0.1)  # Sleep for a short duration before checking again
            
    def printer_callback(self, msg):
        self.printerID = msg.data
        rospy.loginfo(f"Received printerID: {self.printerID}")
    
    def desired_joint_state_callback(self, msg):
        # Extract joint values from the received JointState message
        desired_joint_values = msg.position

        # Move the robot to the desired joint state
        self.move_to_joint_state(desired_joint_values)
    
    def increment_end_effector(self, dx, dy, dz, droll, dpitch, dyaw):
        # Get the current pose
        current_pose = self.group.get_current_pose().pose

        # Increment the position
        current_pose.position.x += dx
        current_pose.position.y += dy
        current_pose.position.z += dz

        # Convert current orientation to roll, pitch, yaw
        roll, pitch, yaw = euler_from_quaternion([current_pose.orientation.x,
                                                  current_pose.orientation.y,
                                                  current_pose.orientation.z,
                                                  current_pose.orientation.w])

        # Increment the orientation (angles are in radians)
        roll += math.radians(droll)
        pitch += math.radians(dpitch)
        yaw += math.radians(dyaw)

        # Convert back to quaternion
        q = quaternion_from_euler(roll, pitch, yaw)
        
        current_pose.orientation.x = q[0]
        current_pose.orientation.y = q[1]
        current_pose.orientation.z = q[2]
        current_pose.orientation.w = q[3]

        # Move the robot
        self.group.set_pose_target(current_pose)
        self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
    
    def end_effector_pose_difference(self, desired_pose):
        """Calculate the difference between current and desired end effector pose."""
        current_pose = self.group.get_current_pose().pose
        position_diff = [
            abs(current_pose.position.x - desired_pose.position.x),
            abs(current_pose.position.y - desired_pose.position.y),
            abs(current_pose.position.z - desired_pose.position.z)
        ]
        orientation_diff = [
            abs(current_pose.orientation.x - desired_pose.orientation.x),
            abs(current_pose.orientation.y - desired_pose.orientation.y),
            abs(current_pose.orientation.z - desired_pose.orientation.z),
            abs(current_pose.orientation.w - desired_pose.orientation.w)
        ]
        print(orientation_diff)
        return position_diff, orientation_diff

    def increment_end_effector_local(self, dx, dy, dz, droll, dpitch, dyaw):
        # Get the current pose of the end effector
        droll = math.radians(droll)
        dpitch = math.radians(dpitch)
        dyaw = math.radians(dyaw)
        
        current_pose = self.group.get_current_pose().pose
            # Convert the current position and orientation of the end effector to a matrix representation
        current_transformation_matrix = np.dot(translation_matrix([current_pose.position.x, current_pose.position.y, current_pose.position.z]), quaternion_matrix([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]))
        
        # Construct the translation matrix for the incremental position
        incremental_translation = translation_matrix([dx, dy, dz])
        
        # Construct the rotation matrix for the incremental rotation
        incremental_rotation = euler_matrix(droll, dpitch, dyaw, 'rxyz')
        
        # Combine the incremental translation and rotation to form the incremental transformation matrix
        incremental_transformation = np.dot(incremental_translation, incremental_rotation)
        
        # Compute the new transformation matrix in the local frame of the end effector by combining the current transformation with the incremental transformation
        new_transformation = np.dot(current_transformation_matrix, incremental_transformation)

        # Extract the updated position and orientation from the new transformation matrix
        new_position = new_transformation[:3, 3]
        new_orientation_quaternion = quaternion_from_matrix(new_transformation)

        # Set the updated position and orientation to the current pose
        current_pose.position.x = new_position[0]
        current_pose.position.y = new_position[1]
        current_pose.position.z = new_position[2]
        current_pose.orientation.x = new_orientation_quaternion[0]
        current_pose.orientation.y = new_orientation_quaternion[1]
        current_pose.orientation.z = new_orientation_quaternion[2]
        current_pose.orientation.w = new_orientation_quaternion[3]

        # Move the robot to the updated pose using MoveIt!
        self.group.set_pose_target(current_pose)
        self.group.go(wait=True)  # Start the motion

        # # Block execution until robot reaches the desired end effector pose
        # pos_tolerance = 0.01  # Tolerance for position (in meters)
        # orient_tolerance = 0.01  # Tolerance for orientation (quaternion values)
        # while True:
        #     position_diff, orientation_diff = self.end_effector_pose_difference(current_pose)
        #     if all(diff < pos_tolerance for diff in position_diff) and all(diff < orient_tolerance for diff in orientation_diff):
        #         break
        #     rospy.sleep(0.1)  # Sleep for a short duration before checking again

        # # Clear the pose targets after reaching the desired pose
        self.group.clear_pose_targets()

    def move_to_joint_state(self, joint_values):
        """
        Move the robot to the specified joint state.

        :param joint_values: List of joint values.
        """
        if len(joint_values) != len(self.group.get_active_joints()):
            rospy.logerr("Provided joint values do not match the number of active joints!")
            return

        # Move to the specified joint state
        self.group.go(joint_values, wait=True)
        # tolerance = 0.01  # Define a suitable tolerance value (in radians) for joint positions
        # while True:
        #     differences = self.joint_states_difference(joint_values)
        #     if all(diff < tolerance for diff in differences):
        #         break
        #     rospy.sleep(0.1)  # Sleep for a short duration before checking again
        self.group.stop()
    
    def joint_states_difference(self, desired_joint_values):
        """Calculate the difference between current and desired joint values."""
        current_joint_values = self.get_current_joint_values()
        print(current_joint_values)
        diff = [abs(a - b) for a, b in zip(current_joint_values, desired_joint_values)]
        print(diff)
        return diff

    def aruco_callback(self, msg):
        if not self.process_aruco_data:
            return
        # Assuming the ID is stored in the header.frame_id of the PoseStamped message
        if msg.pose.position.z > 1.0:
            return
        aruco_id = msg.header.frame_id
        self.aruco_poses[aruco_id] = msg.pose
        
        if msg.pose.position.z > 1.0:
            return
        
        # If the ID is not already present, store the current joint state and the pose of the ArUco tag
        if aruco_id not in self.aruco_ids:
            self.aruco_ids.append(aruco_id)
            current_joint_state = self.group.get_current_joint_values()
            self.joint_states_for_ids[aruco_id] = current_joint_state
            rospy.loginfo(f"Stored joint state and pose for ArUco ID: {aruco_id}")
    
    def get_aruco_pose(self, aruco_id):
        return self.aruco_poses.get(aruco_id, None)
    
    def get_current_joint_values(self):
        return self.latest_joint_values
    
    def joint_state_callback(self, msg):
        # Update the latest joint values
        self.latest_joint_values = msg.position

    def visual_servo_to_tag(self, aruco_id, desired_distance):
        # Get the starting joint state for the given ArUco ID
        start_joint_state = self.joint_states_for_ids.get(str(aruco_id))
        if not start_joint_state:
            rospy.logerr(f"No joint state found for ArUco ID: {aruco_id}")
            return
        # Get the current pose of the ArUco tag
        tag_pose = self.get_aruco_pose(str(aruco_id))
        print("Intial Tag Pose Relative to End Efffector")
        print(tag_pose.position)        
        if not tag_pose:
            rospy.logerr(f"No pose found for ArUco ID: {aruco_id}")
            return

        for x in range(5):
            
            #Sensor Feedback
            tag_pose = self.get_aruco_pose(str(aruco_id))
            
            #Compute Error
            error_x = 0 - tag_pose.position.x
            error_y = 0.08 - tag_pose.position.y
            error_z = tag_pose.position.z - desired_distance
            #Plant Process
            self.increment_end_effector_local(error_x,error_y,error_z,0,0,0)
    
        print("Final Tag Pose Relative to End Efffector")
        print(tag_pose.position)
        
        
    
        
    def scan_plane(self, top_left, side_length, rpy, step_size):
        
        '''
        Scans a square plane using a camera mounted on the robot's end-effector.

        :param top_left: Coordinates of the top-left corner of the square.
        :param side_length: Length of the side of the square.
        :param orientation: Desired orientation of the end-effector during scanning.
        :param step_size: Distance the robot should move between scans. This depends on the camera's FoV.
        '''
        # Convert roll, pitch, yaw to quaternion
        q = quaternion_from_euler(rpy[0], rpy[1], rpy[2])
        
        # Start at the top-left corner
        current_pos = top_left
        
        # Determine the number of steps based on the side length and step size
        num_steps = int(side_length / step_size)
        
        # Determine the scanning direction (left-to-right or right-to-left)
        direction = -1  # 1 for left-to-right, -1 for right-to-left

        for i in range(num_steps):
            # Set the desired position and orientation
            pose = Pose()
            pose.position.x = current_pos[0]
            pose.position.y = current_pos[1] + i * step_size * direction
            pose.position.z = current_pos[2]
            
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            
            
            # Move to the desired pose
            self.group.set_pose_target(pose)
            self.group.go(wait=True)
            self.group.stop()
            self.group.clear_pose_targets()

            # Capture data with the camera (you might want to add a dedicated function or call to do this)

            # Switch direction for the next row
            direction *= -1
            current_pos[1] += step_size  # Move down for the next row
        
if __name__ == '__main__':
    
    controller = RobotController(group_name='tmr_arm')  # Change to your robot's group name
    homepos = [0, 0, 0, 0 , 0, 0]
    scanningStartPos =[math.radians(angle) for angle in [260, 17, -134, 116 , -169, 0]]
    controller.move_to_joint_state(homepos)
    #Start in Scanning Position
    controller.move_to_joint_state(scanningStartPos)
    controller.process_aruco_data = True
    rospy.sleep(5)
    scanningStartPos =[math.radians(angle) for angle in [260, 21, -128, 107 , -169, 0]]
    controller.move_to_joint_state(scanningStartPos)
    controller.visual_servo_to_tag(0,0.1)
    
    rospy.spin()
