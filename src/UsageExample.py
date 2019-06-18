#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from math import pi
import random
import geometry_msgs.msg

#Corobotsim's modules
from scene_manager import sceneManager
from robot_control import robControl

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

def INCHES_TO_MM(measure_in_inches):
    return 25.4*measure_in_inches

def MM_TO_M(measure_in_millimeters):
    return 0.001*measure_in_millimeters

def INCHES_TO_M(measure_in_inches):
    return 0.0254*measure_in_inches

#Add the four walls of a rectangular water tank as collision objects.
#The sides of the tank must be parallel to the reference frame.
#Dx is the distance from the robot base frame to the nearest corner of the tank
#when following the +X direction. Lx is the length of the water tank from that
#corner in the +X direction.
#Dy is the distance from the robot base frame to the nearest corner of the tank
#when following the +Y direction. Ly is the length of the water tank from that
#corner in the +Y direction.
#Dz is the distance from the robot base frame to the nearest corner of the tank
#when following the +Z direction. Lz is the length of the water tank from that
#corner in the +Z direction.
def addWaterTank(Dx, Dy, Dz, Lx, Ly, Lz):
    thickness = 0.01
    c1_size = (abs(Lx), thickness, abs(Lz))
    c2_size = (thickness, abs(Ly), abs(Lz))
    c3_size = (abs(Lx), thickness, abs(Lz))
    c4_size = (thickness, abs(Ly), abs(Lz))

    if(Lx == 0 or Ly == 0 or Lz == 0):
        print("ERROR: A length cannot be zero.")
        return

    sign_x = Lx/abs(Lx)
    sign_y = Ly/abs(Ly)
    sign_z = Lz/abs(Lz)

    c1_position = (Dx+Lx/2,                  Dy+sign_y*thickness/2,     Dz+Lz/2)
    c4_position = (Dx+sign_x*thickness/2,    Dy+Ly/2,                   Dz+Lz/2)
    c3_position = (Dx+Lx/2,                  Dy+Ly-sign_y*thickness/2,  Dz+Lz/2)
    c2_position = (Dx+Lx-sign_x*thickness/2, Dy+Ly/2,                   Dz+Lz/2)

    random_integer = str(random.randint(1,1000))
    sceneMan.addBoxToMoveit(random_integer+"_c1",c1_position, c1_size)
    sceneMan.addBoxToMoveit(random_integer+"_c2",c2_position, c2_size)
    sceneMan.addBoxToMoveit(random_integer+"_c3",c3_position, c3_size)
    sceneMan.addBoxToMoveit(random_integer+"_c4",c4_position, c4_size)


if __name__ == "__main__":
    #Initialization
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('edg_test', anonymous=True)

    #The "arm" planning group as described in coro.srdf represents the UR5
    move_group  = moveit_commander.MoveGroupCommander("manipulator")

    #Manages the objects in the scene
    sceneMan    = sceneManager(rospy, moveit_commander)

    #Manages robot's movements
    robot      = robControl(rospy, move_group)

    #Remove old collision objects from the planning scene
    sceneMan.clear()

    #Add the table under the robot in the scene so MoveIt is aware of it.
    #Table length = 60 in., Table depth = 30 in., Table height = 36.25 in.
    #Position vectors pretty much always describes (X,Y,Z) in this order.
    #The axis are defined by the base frame of the robot relative to which
    #everything is.
    big_table_size      = (INCHES_TO_M(60), INCHES_TO_M(30), INCHES_TO_M(36.25))
    #The position is the one of the center of the box (not a corner) and
    #is relative to the base frame of the robot. Also, the robot base is on
    #top of a 0.5 in. thick plate, is centered along the X axis but offset
    #by 5 in. along the Y axis.
    big_table_position  = (-INCHES_TO_M(60/2 - 5),  INCHES_TO_M(0), -INCHES_TO_M(36.25/2 + 0.5))
    sceneMan.addBoxToMoveit("big_table",big_table_position, big_table_size)

    #Add the other smaller table to the scene.
    #Table length = 36 in., Table depth = 24 in., Table height = 29.25 in.
    small_table_size      = (-INCHES_TO_M(24), INCHES_TO_M(36), INCHES_TO_M(29.25))
    #Center of the table (as a box) relative to the base frame of the robot
    small_table_position  = (INCHES_TO_M(24/2+8.625),  INCHES_TO_M(2), -INCHES_TO_M(29.25/2+7.5))
    sceneMan.addBoxToMoveit("small_table",small_table_position, small_table_size)

    #Add the water tank on top of the big table
    addWaterTank(Dx=INCHES_TO_M(-15), Dy=0, Dz=INCHES_TO_M(-0.5), Lx=INCHES_TO_M(-20), Ly=INCHES_TO_M(-10), Lz=INCHES_TO_M(12.5))

    #Add the water tank on top of the small table
    addWaterTank(Dx=INCHES_TO_M(14), Dy=INCHES_TO_M(9), Dz=INCHES_TO_M(-7.5), Lx=INCHES_TO_M(12), Ly=INCHES_TO_M(-24), Lz=INCHES_TO_M(17))

    #Add a wall to protect the user sitting in front of the computer
    wall_size = (3,0.01,3)
    wall_position = (0,0.9,0)
    sceneMan.addBoxToMoveit("wall",wall_position, wall_size)

    #Request the fingers to go to a specific position
    robot.set_gripper_closing(100,100)

    #Request a RELATIVE position
    robot.set_relative_gripper_closing(100,100)

    #Re-initialize the fingers position
    robot.set_gripper_closing(0,0)

    #Go to the initial position
    # To generate these lines of code, move manually to the desired initial
    # position, then use PrintCurrentPose.py
    pose = Pose()
    pose.position.x = -0.420457239126
    pose.position.y = 0.000219097839434
    pose.position.z = 0.578732831099
    pose.orientation.x = -0.706913700607
    pose.orientation.y = 0.00577487605584
    pose.orientation.z = 0.707256684653
    pose.orientation.w = 0.00525858480106
    robot.goPose(pose)

    #Make a square in the air by moving along the axis of the world frame.
    robot.goRelPosition(goal_pos_rel=(-0.1,0,0))
    robot.goRelPosition(goal_pos_rel=(0,0.1,0))
    robot.goRelPosition(goal_pos_rel=(0.1,0,0))
    robot.goRelPosition(goal_pos_rel=(0,-0.1,0))

    #Show off your orientation skills
    robot.goRelOrientation(wrist_1=-45, wrist_2=0, wrist_3=0)
    robot.goRelOrientation(wrist_1=+45, wrist_2=0, wrist_3=0)

    robot.goRelOrientation(wrist_1=0, wrist_2=-45, wrist_3=0)
    robot.goRelOrientation(wrist_1=0, wrist_2=+45, wrist_3=0)

    robot.goRelOrientation(wrist_1=0, wrist_2=0, wrist_3=-45)
    robot.goRelOrientation(wrist_1=0, wrist_2=0, wrist_3=+45)
