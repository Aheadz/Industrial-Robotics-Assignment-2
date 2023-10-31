#! /usr/bin/python3

# Import Library Needed:
import numpy as np
import rospy
import time
import actionlib
import moveit_msgs.msg
import moveit_commander
import math
import sys
import swift
import roboticstoolbox as rtb
import copy

# Python Message via ROS:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from spatialmath import SE3
from sensor_msgs.msg import JointState
from math import pi
from scipy.spatial.transform import Rotation as R
from spatialmath.base import *
from spatialmath import SE3
import geometry_msgs.msg

from eulerQuaternion import *
from geometry import *

class UR3e:

    def __init__(self):
        # Define a Subcriber:
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.getpos)

        # First initialize moveit_commander and rospy.
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('gripperAndRobotPublisher', anonymous=True)
        # Instantiate a RobotCommander object.  This object is an interface to the robot as a whole.
        robot = moveit_commander.RobotCommander()
        # Instantiate a PlanningSceneInterface object.  This object is an interface to the world surrounding the robot.
        scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a MoveGroupCommander object. This object is an interface to one group of joints. This interface can be used to plan and execute motions on the arm.
        self.group = moveit_commander.MoveGroupCommander("ur3e_group")

        # We create this DisplayTrajectory publisher which is used below to publish trajectories for RVIZ to visualize.
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        # Optional:
        self.group.set_max_velocity_scaling_factor(0.3)
        self.group.set_max_acceleration_scaling_factor(1)
        self.group.set_planning_time(10)
        self.group.set_num_planning_attempts(3)
        self.group.allow_replanning(1)

        self.currentQ = []

        time.sleep(2)


    def get_position(self):
        return self.currentQ


    # ---- CallBack Function:
    def getpos(self, msg):
        self.currentQ = msg.position

    
    def get_joint_list(self) -> list:

        joint_list = []  # List of joints
        j = 0
        for link in self.model.links:
            if link.isjoint:
                joint_list.append(j)
            j += 1

        return joint_list


    # ---- get_link_transform function:
    def get_link_transform(self, q) -> list:
        transform_list = []  # Tranforms array of link
        joint_list = self.get_joint_list()  # List of joints as link index

        for i in joint_list:
            transform_list.append(self.model.fkine(
                q, end=self.model.links[i].name))
        return transform_list


    def returnOrderedJoints(self):
        while len(self.currentQ) != 6:
            pass

        new_q = list(self.currentQ)
        # Perform the element assignment
        new_q[0], new_q[2] = new_q[2], new_q[0]

        return new_q


    def waitForRobotToFinishMoving(self, q):
        # This code waits for the robot to stop moving, by checking the joint differencem I can check one more time just incase, with a break
        while True:
            # print("\nWaiting until joint angles are the same")
            joint_difference = 0
            temp_q = self.returnOrderedJoints()
            for i in range(6):
                joint_difference += abs(temp_q[i]-q[i])
            if joint_difference <= 0.001:
                break

        time.sleep(0.1)
        print("Robot has finished moving")


    def getInfo(self):
        current_q = self.returnOrderedJoints()
        print("Current Joint State after being rearranged:", current_q)

        current_pose = self.model.fkine(self.currentQ).A
        print("\n\nCurrent Pose: \n", current_pose)

        rotation_matrix = current_pose[:3, :3]

        # Create a Rotation object from the rotation matrix
        r = R.from_matrix(rotation_matrix)

        # Extract the roll, pitch, and yaw angles in radians
        roll, pitch, yaw = r.as_euler('zyx', degrees=False)

        # Print the roll, pitch, and yaw angles
        print("Roll (radians):", roll)
        print("Pitch (radians):", pitch)
        print("Yaw (radians):", yaw)

        time.sleep(2)

        return


    def returnCurrentPose(self):
        current_pose = self.model.fkine(self.returnOrderedJoints()).A
        return current_pose


    def jointsAndEndEffectorInformation(self):
        # Where the UR3E raw end effector is:
        print("\n============ End Effector Location of Wrist: ")
        pose = self.group.get_current_pose(end_effector_link="wrist_3_link").pose
        print(pose)
        print("\nWith Euler Angles:")
        print(euler_from_quaternion((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)))

        # Where the gripper end effector is (roughly cos it assumes a fixed poition, not the actual position):
        print("\n============ End Effector Location of Left Finger Link: ")
        pose = self.group.get_current_pose(end_effector_link="rg2_l_finger_link").pose
        print(pose)
        print("\nWith Euler Angles:")
        print(euler_from_quaternion((pose.orientation.x,pose.orientation.y, pose.orientation.z, pose.orientation.w)))

        # current_state.getJointStateGroup(group)
        joint_states_vector = self.group.get_current_joint_values()
        for i in range(len(joint_states_vector)):
            print("Joint " + str(i) + " is at: " + str(joint_states_vector[i]) + " radians")


    # desiredSetPosition = either home OR zeros
    def movingRobotToSetPoses(self, desiredSetPosition):
        print("============ Going to '" + desiredSetPosition + "' pose\n")
        self.group.set_named_target(desiredSetPosition)

        # Now, we call the planner to compute the plan and visualize it if successful
        plan1 = self.group.plan()
        rospy.sleep(0.1)
        # To move execute in moveit, and therefore move in Gazebo
        self.group.go(wait=True)


    # Joint being changed is from 0-5, and desiredJointPosition is in radians
    def changeSpecificJoint(self, joint_being_changed, desiredJointPosition):
        joint_states_vector = self.group.get_current_joint_values()

        print("changing joint " + str(joint_being_changed) + " which is currently at: " + str(joint_states_vector[joint_being_changed]) + " to: " + str(desiredJointPosition))
        joint_states_vector[joint_being_changed] = desiredJointPosition
        self.group.set_joint_value_target(joint_states_vector)
        self.group.go(wait=True)


    # offset specific joint, by reading current position
    def offsetSpecificJoint(self, joint_being_changed, offset):

        joint_states_vector = self.group.get_current_joint_values()

        print("Joint state Vector: ")
        print(joint_states_vector)

        desiredJointPosition = joint_states_vector[joint_being_changed]+offset

        while (desiredJointPosition > pi):
            desiredJointPosition = desiredJointPosition - 2*pi
        while (desiredJointPosition < -pi):
            desiredJointPosition = desiredJointPosition + 2*pi

        print("changing joint " + str(joint_being_changed) + " which is currently at: " + str(joint_states_vector[joint_being_changed]) + " by: " + str(offset) + " so final position is at:" + str(desiredJointPosition))

        joint_states_vector[joint_being_changed] = desiredJointPosition
        print("Joint state Vector: ")
        print(joint_states_vector)

        self.group.set_joint_value_target(joint_states_vector)
        self.group.go(wait=True)

        rospy.sleep(0.01)


    # all 6 entered variables are the desired joint position values in radians
    def changeAllJoints(self, dj0, dj1, dj2, dj3, dj4, dj5):
        joint_states_vector = self.group.get_joint_value_target()
        print("Original Joints:")
        for i in joint_states_vector:
            # print(str(joint_states_vector[i]))
            print(str(i))

        joint_states_vector = [dj0, dj1, dj2, dj3, dj4, dj5]
        print("Change Joints to:")
        for i in joint_states_vector:
            # print(str(joint_states_vector[i]))
            print(str(i))

        self.group.set_joint_value_target(joint_states_vector)
        self.group.go(wait=True)


    def cartesianPathPlanning(self):
        waypoints = []
        scale = 0.5
        pose = self.group.get_current_pose().pose
        pose.position.x -= scale * -0.2
        waypoints.append(copy.deepcopy(pose))
        pose.position.z += scale * -0.3
        waypoints.append(copy.deepcopy(pose))
        pose.position.z -= scale * -0.1
        pose.position.y -= scale * -1.5
        waypoints.append(copy.deepcopy(pose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm which is why we will specify 0.01 as
        # the eef_step in Cartesian translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient for most cases
        (plan, fraction) = self.group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold
        print("\n\nWaypoints:")
        print(waypoints)

        print("Executing:")
        self.group.execute(plan, wait=True)


    def jogEndEffector(self, x, y, z):

        # Where the UR3E raw end effector is:
        print("\n\n============ End Effector Location of Wrist: ")
        pose = self.group.get_current_pose(end_effector_link="wrist_3_link").pose

        print("\nPose Before:")
        print(pose)

        print("\nPose After:")
        newPose = translate_pose_msg(pose, x, y, z)
        print(newPose)

        self.group.set_pose_target(newPose)
        self.group.go(wait=True)


    # RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
    def setEndEffectorWithOrientation(self, x, y, z, roll, pitch, yaw):
        pose_target = geometry_msgs.msg.Pose()

        q = get_quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z
        self.group.set_pose_target(pose_target)
        self.group.go(wait=True)


    # RPY: to convert: 90deg, 0, -90deg, enter: 1.5707, -1.5707,1.5707
    def setEndEffectorOfFingerWithOrientation(self, gripper_length, x, y, z, roll, pitch, yaw):

        pose_target = geometry_msgs.msg.Pose()

        q = get_quaternion_from_euler(roll, pitch, yaw)
        pose_target.orientation.x = q[0]
        pose_target.orientation.y = q[1]
        pose_target.orientation.z = q[2]
        pose_target.orientation.w = q[3]

        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        pose_target = translate_pose_msg(pose_target, 0, 0, -gripper_length)

        self.group.set_pose_target(pose_target)
        self.group.go(wait=True)