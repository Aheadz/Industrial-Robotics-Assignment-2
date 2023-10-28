#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_keyboard_control', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "tmr_arm"  # Change this to your group name
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Print current pose
    current_pose = move_group.get_current_pose().pose
    rospy.loginfo("Current Pose:")
    rospy.loginfo(pose_to_list(current_pose))

    print("Commands:")
    print("w/a/s/d: Translate end-effector in x/y plane")
    print("r/f: Translate end-effector in z plane")
    print("q: Quit")

    while not rospy.is_shutdown():
        key = input("Enter command: ")

        if key == 'w':
            current_pose.position.x += 0.05
        elif key == 'a':
            current_pose.position.y -= 0.05
        elif key == 's':
            current_pose.position.x -= 0.05
        elif key == 'd':
            current_pose.position.y += 0.05
        elif key == 'r':
            current_pose.position.z += 0.05
        elif key == 'f':
            current_pose.position.z -= 0.05
        elif key == 'q':
            print("Quitting...")
            break
        else:
            print("Unknown command")
            continue

        # Plan and execute
        move_group.set_pose_target(current_pose)
        plan = move_group.plan()
        move_group.execute(plan, wait=True)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
