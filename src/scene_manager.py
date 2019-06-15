# This modules manages the simulated Gazebo scene
# by adding, removing or modifying objects

from copy import deepcopy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
import time

class sceneManager:
    #Constructor needs to receive a properly initialized rospy instance
    def __init__(self, rospy_instance, moveit_commander):
        self.rospy = rospy_instance
        self.moveit_robotCmd = moveit_commander.RobotCommander()
        self.moveit_scene = moveit_commander.PlanningSceneInterface()
        #DO NOT REMOVE this delay, its needed to let the PlanningSceneInterface initialize.
        self.rospy.sleep(1)

    #Remove all collision objects from the planning scene
    def clear(self):
        self.moveit_scene. remove_world_object()

    #Add a collision box around the table so MoveIt finds
    #a way around it.
    def addBoxToMoveit(self, name, position=(0,0,0), size=(0, 0, 0)):
        box_pose = PoseStamped()
        box_pose.header.frame_id = self.moveit_robotCmd.get_planning_frame()
        box_pose.pose.position.x = position[0]
        box_pose.pose.position.y = position[1]
        box_pose.pose.position.z = position[2]

        self.moveit_scene.add_box(name, box_pose, size=size)
        #DO NOT REMOVE this delay, its needed
        # to let the PlanningSceneInterface update.
        time.sleep(1)

        print("Objects added to MoveIt planning scene:")
        print(self.moveit_scene.get_known_object_names())
