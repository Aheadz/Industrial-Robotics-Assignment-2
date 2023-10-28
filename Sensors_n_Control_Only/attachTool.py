import rospy
import moveit_commander
import sys
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import JointState
import math

class RobotController:
    def __init__(self, group_name='tmr_arm'):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('visual_servoing_program', anonymous=True)
        
        
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
        
    def printer_callback(self, msg):
        self.printerID = msg.data
        rospy.loginfo(f"Received printerID: {self.printerID}")
    
    def desired_joint_state_callback(self, msg):
        # Extract joint values from the received JointState message
        desired_joint_values = msg.position

        # Move the robot to the desired joint state
        self.move_to_joint_state(desired_joint_values)
        
        self.move_to_joint_state(desired_joint_values)
    
    def increment_end_effector(self, dx, dy, dz, droll, dpitch, dyaw):
        # Get the current pose
        current_pose = self.group.get_current_pose().pose

        # Increment the position
        current_pose.position.x += dx
        current_pose.position.y += dy
        current_pose.position.z += dz

        # Convert current orientation to roll, pitch, yaw
        from tf.transformations import euler_from_quaternion, quaternion_from_euler
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
        self.group.stop()
    
    def aruco_callback(self, msg):
        # Assuming the ID is stored in the header.frame_id of the PoseStamped message
        aruco_id = msg.header.frame_id

        # If the ID is not already present, store the current joint state and the pose of the ArUco tag
        if aruco_id not in self.aruco_ids:
            self.aruco_ids.append(aruco_id)
            current_joint_state = self.group.get_current_joint_values()
            self.joint_states_for_ids[aruco_id] = current_joint_state
            self.aruco_poses[aruco_id] = msg.pose
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
        start_joint_state = self.joint_states_for_ids.get(aruco_id)
        if not start_joint_state:
            rospy.logerr(f"No joint state found for ArUco ID: {aruco_id}")
            return

        # Set the robot to the starting joint state
        self.move_to_joint_state(start_joint_state)

        # Get the current pose of the ArUco tag
        tag_pose = self.get_aruco_pose(aruco_id)
        if not tag_pose:
            rospy.logerr(f"No pose found for ArUco ID: {aruco_id}")
            return

        # Compute the error
        error_x = tag_pose.position.x
        error_y = tag_pose.position.y
        error_z = tag_pose.position.z - desired_distance

        # Compute the control input using PID
        control_input_x = self.kp[0] * error_x + self.ki[0] * self.error_integral[0] + self.kd[0] * (error_x - self.prev_error[0])
        control_input_y = self.kp[1] * error_y + self.ki[1] * self.error_integral[1] + self.kd[1] * (error_y - self.prev_error[1])
        control_input_z = self.kp[2] * error_z + self.ki[2] * self.error_integral[2] + self.kd[2] * (error_z - self.prev_error[2])

        # Update the error accumulators and previous error
        self.error_integral[0] += error_x
        self.error_integral[1] += error_y
        self.error_integral[2] += error_z
        self.prev_error = [error_x, error_y, error_z]

        # Use the control input to move the robot (this part might need more details on how you want to apply the control input)
        # For example, you might want to use the control input to adjust the robot's joint values or end-effector pose
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
        direction = 1  # 1 for left-to-right, -1 for right-to-left

        for i in range(num_steps):
            # Set the desired position and orientation
            pose = Pose()
            pose.position.x = current_pos[0] + i * step_size * direction
            pose.position.y = current_pos[1]
            pose.position.z = current_pos[2]
            pose.orientation = q
            
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
    scanningStartPos = [0, 0, 0, 0 , 0, 0]
    #Home Robot
    controller.move_to_joint_state(homepos)
    #Start in Scanning Position
    controller.move_to_joint_state(scanningStartPos)
    endEffectorOrientation = [math.radians(0), math.radians(90), math.radians(45)]  # Just an example
    controller.scan_plane(controller.group.get_current_pose(),0.5,endEffectorOrientation,0.05)
    
    
    rospy.spin()
