#!/usr/bin/env python
import math
import numpy as np
from scipy import linalg, optimize
import matplotlib.pyplot as plt
from spatialmath import *
from spatialmath.base import *
from spatialmath.base import sym
import spatialgeometry as geometry
import roboticstoolbox as rtb
import swift
import rospy
from sensor_msgs.msg import JointState

ur5 = 0
env = 0

#Modify pose of objects

def robot_callback(data):
    if data.name[0] == "UR5":
        rospy.loginfo("Received joint state for %s:", data.name[0])
        q1 = np.array(data.position, dtype=float)
        ur5.q = q1
        env.step()
        print(q1)
    elif data.name[0] == "TM5":
        rospy.loginfo("Received joint state for %s:", data.name[0])
    

def listener():
    rospy.init_node('joint_state_subscriber', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, robot_callback)
    rospy.spin()
    
def initialize_environment():
    global env, ur5
    env = swift.Swift()
    env.launch(realtime = True)
    garagePath = '/home/strangelove/Desktop/Spring2023/Auto3DPrintFarm/UTS-IR-SnC-Project-Spring2023/PY_Env_1/Garage.dae'
    garage = geometry.Mesh(garagePath)
    env.add(garage)


if __name__ == '__main__':
    initialize_environment()
    listener()