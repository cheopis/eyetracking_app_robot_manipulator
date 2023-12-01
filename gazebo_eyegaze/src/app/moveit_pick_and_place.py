#!/usr/bin/env python

import sys
import rospy
import tf2_ros
import numpy as np
import json
import time

from math import pi
from moveit_commander.planning_scene_interface import PlanningSceneInterface
from moveit_commander.roscpp_initializer import roscpp_initialize, roscpp_shutdown
from moveit_commander.move_group import MoveGroupCommander
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.msg import Grasp, PlaceLocation

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from gazebo_msgs.srv import GetModelState
import geometry_msgs.msg
import franka_gripper.msg
import geometry_msgs.msg
import pdb
import copy
import shape_msgs.msg
import visualization_msgs

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler

GROUP_NAME_ARM = 'panda_arm'
GROUP_NAME_GRIPPER = 'panda_hand'

GRIPPER_FRAME = "panda_link8" #"panda_link8"
#GRIPPER_FRAME = "panda_gripper_center"  # if gripper_center = true
GRIPPER_JOINT_NAMES = ['panda_finger_joint1','panda_finger_joint2']
GRIPPER_EFFORT = [200.0]

REFERENCE_FRAME = "world"
ARM_BASE_FRAME = "panda_link0"

def openGripper():
  t = JointTrajectory()

  # Set the joint names to the gripper joint names
  t.header.stamp = rospy.get_rostime()
  t.joint_names = GRIPPER_JOINT_NAMES

  # Initialize a joint trajectory point to represent the goal
  tp = JointTrajectoryPoint()
  # Set them as open, wide enough for the object to fit. 
  tp.positions = [0.04, 0.04]
  tp.effort = GRIPPER_EFFORT
  tp.velocities = [0.1, 0.1]
  tp.accelerations = [0.1, 0.1]
  tp.time_from_start = rospy.Duration(5.)

  t.points.append(tp)
  return t

def closedGripper():
  t = JointTrajectory()

  # Set the joint names to the gripper joint names
  t.header.stamp = rospy.get_rostime()
  t.joint_names = GRIPPER_JOINT_NAMES

  # Initialize a joint trajectory point to represent the goal
  tp = JointTrajectoryPoint()
  # Set them as open, wide enough for the object to fit. 
  tp.positions = [0.00, 0.00]
  tp.effort = [100]
  tp.velocities = [0.1, 0.1]
  tp.accelerations = [0.1, 0.1]
  tp.time_from_start = rospy.Duration(5.)
  t.points.append(tp)
  return t

def show_gazebo_models(item):
  try:
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    blockName = item
    resp_coordinates = model_coordinates(blockName,"world")
    position = resp_coordinates.pose.position
    orientation = resp_coordinates.pose.orientation

  except rospy.ServiceException as e:
    rospy.loginfo("Get Model State service call failed:  {0}".format(e))
      
  return(position,orientation)

def pick(move_group, target_pose, target_name):
  # Create a vector of grasps to be attempted, currently only creating single grasp.
  g = Grasp()
  # Setting grasp pose
  # ++++++++++++++++++++++
  # This is the pose of end-effector panda_link8. |br|
  # From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  # of the cube). |br|
  # Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  # extra padding)
  g.grasp_pose.header.frame_id = REFERENCE_FRAME
  g.grasp_pose.pose.position.x = target_pose[0] #0.4 - 0.06 #0.415 
  g.grasp_pose.pose.position.y = target_pose[1] #0
  g.grasp_pose.pose.position.z = target_pose[2] #0.45

  q = quaternion_from_euler(-pi/2, -pi/4, -pi/2) # vertical
  #q = quaternion_from_euler(pi/2, -pi/4, pi/2) # horinzontal
  g.grasp_pose.pose.orientation.x = q[0]
  g.grasp_pose.pose.orientation.y = q[1]
  g.grasp_pose.pose.orientation.z = q[2]
  g.grasp_pose.pose.orientation.w = q[3]

  # Setting pre-grasp approach
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  g.pre_grasp_approach.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as positive x axis 
  g.pre_grasp_approach.direction.vector.x = 1.0
  g.pre_grasp_approach.min_distance = 0.05
  g.pre_grasp_approach.desired_distance = 0.13

  g.max_contact_force = 5
  # Setting post-grasp retreat
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  g.post_grasp_retreat.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as positive z axis 
  g.post_grasp_retreat.direction.vector.z = 1.0
  g.post_grasp_retreat.min_distance = 0.15
  g.post_grasp_retreat.desired_distance = 0.17
  g.id = str('grasp')

  g.allowed_touch_objects = target_name
  # Set support surface as table1.

  # Setting posture of eef before grasp
  # +++++++++++++++++++++++++++++++++++
  g.pre_grasp_posture = openGripper()

  # Setting posture of eef during grasp
  # +++++++++++++++++++++++++++++++++++
  g.grasp_posture = closedGripper()
  move_group.set_support_surface_name("table1")
  # Call pick to pick up the object using the grasps given
  result = move_group.pick(target_name, g)
  #print(result)
  #return(result)

def place(group, name):
  # BEGIN_SUB_TUTORIAL place
  # TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  # location in
  # verbose mode." This is a known issue and we are working on fixing it. |br|
  # Create a vector of placings to be attempted, currently only creating single place location.
  place_location = PlaceLocation()

  # Setting place location pose
  # +++++++++++++++++++++++++++
  place_location.place_pose.header.frame_id = REFERENCE_FRAME
  q = quaternion_from_euler(0, 0, pi/2)
  place_location.place_pose.pose.orientation.x = q[0]
  place_location.place_pose.pose.orientation.y = q[1]
  place_location.place_pose.pose.orientation.z = q[2]
  place_location.place_pose.pose.orientation.w = q[3]

  # While placing it is the exact location of the center of the object. 
  place_location.place_pose.pose.position.x = 0
  place_location.place_pose.pose.position.y = 0.6
  place_location.place_pose.pose.position.z = 0.43

  # Setting pre-place approach
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  place_location.pre_place_approach.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as negative z axis 
  place_location.pre_place_approach.direction.vector.z = -1.0
  place_location.pre_place_approach.min_distance = 0.095
  place_location.pre_place_approach.desired_distance = 0.115

  # Setting post-grasp retreat
  # ++++++++++++++++++++++++++
  # Defined with respect to frame_id 
  place_location.post_place_retreat.direction.header.frame_id = REFERENCE_FRAME
  # Direction is set as negative y axis 
  place_location.post_place_retreat.direction.vector.y = -1.0
  place_location.post_place_retreat.min_distance = 0.05
  place_location.post_place_retreat.desired_distance = 0.15

  #Setting posture of eef after placing object
  # +++++++++++++++++++++++++++++++++++++++++++
  # Similar to the pick case 
  place_location.post_place_posture = openGripper()

  # Set support surface as table2.
  group.set_support_surface_name("table2")
  # Call place to place the object using the place locations given.
  group.place(name, place_location)


def create_collision_object(shape_type,pos,size,frame_id,op,object_id):
  col = CollisionObject()
  col.id = object_id
  col.operation = op
  col.header.frame_id=frame_id

  # create primitive
  primitive = SolidPrimitive()
  primitive.type = shape_type
  primitive.dimensions = size

  # create pose
  pose = Pose()
  pose.position.x = pos[0]
  pose.position.y = pos[1]
  pose.position.z = pos[2]
  pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
  pose.orientation.w = 1


  col.primitives = [primitive]
  col.primitive_poses = [pose]

  return col

def addCollisionObjects(planning_scene_interface):
  #Creating Environment
  # ^^^^^^^^^^^^^^^^^^^^
  # Create collision objects.
  planning_scene_interface.add_object(create_collision_object(
                              1,
                              [0.6, 0, 0.2],   # place
                              [0.2, 0.4, 0.4], # size
                              REFERENCE_FRAME,
                              0,
                              "table1" ))

  planning_scene_interface.add_object(create_collision_object(
                              1,
                              [0.0, 0.7, 0.2],
                              [0.4, 0.2, 0.4],
                              REFERENCE_FRAME,
                              0,
                              "table2" ))
 
  # Read the json file to get the items sizes.
  file = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json", "r")
  content = file.read()
  data = json.loads(content)

  for item in data["yolo"]["items"]:
    holder_pos = np.array(item["pos"]) - np.array((0.8, 0.0, 0.0))
    planning_scene_interface.add_object(create_collision_object(
                                1,
                                item["pos"],
                                data["items_sizes"][item["type"]],
                                REFERENCE_FRAME,
                                0,
                                item["name"]))
  '''
    planning_scene_interface.add_object(create_collision_object(
                                1,
                                [0.58, 0.0, 0.45],
                                [0.06, 0.06, 0.06],#data["items_sizes"]["holder"],
                                REFERENCE_FRAME,
                                0,
                                item["name"]))
  '''
  file.close()

def removeCollisionObjects(planning_scene_interface):
  try:
    planning_scene_interface.remove_world_object("table1")
    planning_scene_interface.remove_world_object("table2")
    planning_scene_interface.remove_world_object('apple_01')
  except:
    pass

def MoveItPanda(target_pose, name):
  rospy.sleep(1.0) 

  start = time.time()

  planning_scene_interface = PlanningSceneInterface(synchronous=True)
  group = MoveGroupCommander(GROUP_NAME_ARM)
  group.set_end_effector_link(GRIPPER_FRAME)
  group.set_planning_time(30.0)

  removeCollisionObjects(planning_scene_interface)
  addCollisionObjects(planning_scene_interface)

  group_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
  group.set_named_target("ready")
  group_gripper.set_named_target("close")
  group.go(wait=True)
  group_gripper.go(wait=True)

  # Wait a bit for ROS things to initialize
  rospy.sleep(1.0)
  pick(group,target_pose, name)

  #q = quaternion_from_euler(-pi/2, -pi/4, 0)
  #pos = [0, 0.5, 0.6]
  #go_to(group, pos, q)

  rospy.sleep(1.0)
  place(group, name)
  #group_gripper.set_named_target("open")
  #group_gripper.go(wait=True)
  end = time.time()
  '''
  f = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/ppmoveit_data.txt", "a")
  
  item = show_gazebo_models('box_apple')[0]
  end_position = np.array([item.x,item.y,item.z])
  f.writelines([str(end - start),";", str(end_position),"\n"])
  f.writelines([str(end - start),"\n"])
  '''
  roscpp_shutdown()

if __name__ == "__main__":
  roscpp_initialize(sys.argv)
  rospy.init_node("panda_arm_pick_place_python")
  pose = (0.4755737699, 0.0016393442, 0.44)

  pose_c = (0.5743647543125, -0.147540978, 0.43)
  pose_a = (0.57385245925, -0.0005122950624999999, 0.43)
  pose_b = (0.5723155740625, 0.148565568125, 0.43)

  MoveItPanda(pose_a, "apple_02")