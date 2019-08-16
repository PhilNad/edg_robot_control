#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi, radians
import random
import tf2_ros
from tf.transformations import *

#Corobotsim's modules
from pose_utils import poseUtilities


if __name__ == "__main__":
    #Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('edg_print_pose', anonymous=True)

    #The "arm" planning group as described in coro.srdf represents the UR5
    move_group  = moveit_commander.MoveGroupCommander("manipulator")

    poseUtils      = poseUtilities(rospy,move_group, WORLD_TO_BASE=(0,0,0), FLANGE_TO_FINGERTIP=0.00)

    #Print the current pose
    pose = poseUtils.printCurrentPose()
    joints_position = move_group.get_current_joint_values()

    print("Plug-and-Play Code (cartesian-space, using path planning):")
    print("\tpose = Pose()")
    print("\tpose.position.x = "+str(pose.position.x))
    print("\tpose.position.y = "+str(pose.position.y))
    print("\tpose.position.z = "+str(pose.position.z))
    print("\tpose.orientation.x = "+str(pose.orientation.x))
    print("\tpose.orientation.y = "+str(pose.orientation.y))
    print("\tpose.orientation.z = "+str(pose.orientation.z))
    print("\tpose.orientation.w = "+str(pose.orientation.w))
    print("\trobot.goPose(pose)")

    print("Plug-and-Play Code (joints-space, no collision avoidance!):")
    print("\tjoint_values = ["+str(joints_position[0])+","+str(joints_position[1])+","+str(joints_position[2])+","+str(joints_position[3])+","+str(joints_position[4])+","+str(joints_position[5])+"]")
    print("\tfor i in range(0,10):")
    print("\t\tmove_group.go(joint_values, wait=True)")
