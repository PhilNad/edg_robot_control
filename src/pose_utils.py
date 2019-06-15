#This module provides a set of utilities to manage
#poses, frames and transformation matrices

from math import pi,radians,degrees
#The following import represents this
#https://www.lfd.uci.edu/~gohlke/code/transformations.py.html
from tf.transformations import *
from geometry_msgs.msg import Pose

class poseUtilities:
    def __init__(self, rospy_instance, move_group, WORLD_TO_BASE=(0,0,0), FLANGE_TO_FINGERTIP=0.04):
        self.rospy = rospy_instance
        self.move_group = move_group
        #To be manually measured on the robot, see makeRelToolPose()
        self.FLANGE_TO_FINGERTIP = FLANGE_TO_FINGERTIP
        self.WORLD_TO_BASE       = WORLD_TO_BASE

    #Create a pose relative to the end-effector current pose given
    # a position difference XYZ and an orientation Euler XYZ angle difference
    def makeIncrementalEEPose(self, position=(0,0,0), orientation=(0,0,0)):
        pose = self.move_group.get_current_pose().pose

        #We need to translate the orientation in Euler angles first
        orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orient_list)

        #Then we can add it up with the given Euler angles increments
        quat = quaternion_from_euler(roll+radians(orientation[0]), pitch+radians(orientation[1]), yaw+radians(orientation[2]))

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x += position[0]
        pose.position.y += position[1]
        pose.position.z += position[2]
        return pose

    #Create a pose given a XYZ position and Euler XYZ angles orientation
    def makePose(self, position=(0.4,0,0.1), orientation=(0,0,0)):
        pose = Pose()
        quat = quaternion_from_euler(radians(orientation[0]), radians(orientation[1]), radians(orientation[2]))
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        return pose

    #Describe a goal pose to reach with our end_effector
    #The position is described in an euclidian form (x,y,z)
    #The orientation is described as a quaternion (x,y,z,w)
    #https://docs.ros.org/melodic/api/geometry_msgs/html/msg/Pose.html
    def printPose(self, pose):
        orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orient_list)
        roll    = degrees(roll)
        pitch   = degrees(pitch)
        yaw     = degrees(yaw)

        print("Position")
        print("\tX: "+str(pose.position.x)+"\t M")
        print("\tY: "+str(pose.position.y)+"\t M")
        print("\tZ: "+str(pose.position.z)+"\t M")
        print("Quaternion orientation")
        print("\tX: "+str(pose.orientation.x))
        print("\tY: "+str(pose.orientation.y))
        print("\tZ: "+str(pose.orientation.z))
        print("\tW: "+str(pose.orientation.w))
        print("Euler Orientation:")
        print("\tRoll  X: "+str(round(roll,1))+"\t DEG")
        print("\tPitch Y: "+str(round(pitch,1))+"\t DEG")
        print("\tYaw   Z: "+str(round(yaw,1))+"\t DEG")


    def printCurrentPose(self):
        print("The EE link is: " + self.move_group.get_end_effector_link())
        print("The Planning frame is: " + self.move_group.get_planning_frame())
        current_pose = self.move_group.get_current_pose().pose
        self.printPose(current_pose)
        return current_pose
