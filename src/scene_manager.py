# This modules manages the virtual MoveIt scene
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
        self.addBoxToMoveit(random_integer+"_c1",c1_position, c1_size)
        self.addBoxToMoveit(random_integer+"_c2",c2_position, c2_size)
        self.addBoxToMoveit(random_integer+"_c3",c3_position, c3_size)
        self.addBoxToMoveit(random_integer+"_c4",c4_position, c4_size)
