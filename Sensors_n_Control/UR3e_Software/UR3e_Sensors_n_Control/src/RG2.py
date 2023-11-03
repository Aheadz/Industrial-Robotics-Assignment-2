from onrobot_rg_control.msg import OnRobotRGOutput
import rospy
import time


class RG2:

    def __init__(self):
        self.depth= 0.220
        self.gripper_publish = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
        time.sleep(1)


    def moveGripper(self, position):  # enter position when callling function

        command = OnRobotRGOutput()
        max_width = 1100
        command.rGFR = 400
        command.rGWD = min(max_width, position)
        command.rCTR = 16
        self.gripper_publish.publish(command)
        rospy.sleep(0.01)
        print("\n\n\nMoving Gripper To Position:" + str(position) + "\n\n")
        
        time.sleep(1)
        
        self.depth = (-2E-08*(position**3) - 6E-06*(position**2) - 0.0085*(position**1) + 214.81+ 13)/1000