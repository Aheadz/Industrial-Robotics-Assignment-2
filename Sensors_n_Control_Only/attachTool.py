import rospy
import moveit_commander
import tkinter as tk
from tkinter import DoubleVar
import sys

class RobotController:
    def __init__(self, group_name='tmr_arm'):
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('robot_slider_controller', anonymous=True)

        # Create a MoveGroupCommander object
        self.group = moveit_commander.MoveGroupCommander(group_name)

    def set_joint_positions(self, joint_values):
        self.group.go(joint_values, wait=True)
        self.group.stop()

class RobotControlGUI:
    def __init__(self, robot_controller):
        self.robot_controller = robot_controller

        # Create main window
        self.root = tk.Tk()
        self.root.title("6DOF Robot Controller")

        # Create sliders
        self.joint_vars = [DoubleVar() for _ in range(6)]
        for i in range(6):
            tk.Scale(self.root, from_=-3.14, to=3.14, orient=tk.HORIZONTAL,
                     label=f'Joint {i + 1}', variable=self.joint_vars[i],
                     command=self.update_robot).pack()

    def run(self):
        self.root.mainloop()

    def update_robot(self, _):
        joint_values = [var.get() for var in self.joint_vars]
        self.robot_controller.set_joint_positions(joint_values)

if __name__ == '__main__':
    controller = RobotController(group_name='tmr_arm')  # Change to your robot's group name
    gui = RobotControlGUI(controller)
    gui.run()
