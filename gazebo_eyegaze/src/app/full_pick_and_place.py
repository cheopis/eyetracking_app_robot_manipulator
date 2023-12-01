#!/usr/bin/env python

import sys
import rospy
import numpy as np
import json
import time

from math import pi
from moveit_commander.roscpp_initializer import roscpp_initialize, roscpp_shutdown

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from shape_msgs.msg import SolidPrimitive
from gazebo_msgs.srv import GetModelState

import visualization_msgs
import copy
from tf.transformations import quaternion_from_euler
import std_msgs.msg

from panda.gripper import GripperInterface

GROUP_NAME_ARM = 'panda_arm'
GROUP_NAME_GRIPPER = 'panda_hand'

GRIPPER_FRAME = "panda_link8"
GRIPPER_JOINT_NAMES = ['panda_finger_joint1','panda_finger_joint2']
GRIPPER_EFFORT = [200.0]

REFERENCE_FRAME = "world"
ARM_BASE_FRAME = "panda_link0"

COLOR_TRANSLUCENT = std_msgs.msg.ColorRGBA(0.0, 0.0, 0.0, 0.5)

class PickAndPlacePanda(object):
  def __init__(self, group_name=GROUP_NAME_ARM):
    moveit_commander.roscpp_initialize(sys.argv)
    self.robot = moveit_commander.RobotCommander()
    self.move_group = moveit_commander.MoveGroupCommander(group_name)
    self.scene = moveit_commander.PlanningSceneInterface()

    self.move_group.set_end_effector_link(GRIPPER_FRAME)
    # Create a publisher to visualize the position constraints in Rviz
    self.marker_publisher = rospy.Publisher(
        "/visualization_marker", visualization_msgs.msg.Marker, queue_size=20,
    )
    rospy.sleep(0.5)  # publisher needs some time to context Rviz
    self.remove_all_markers()
    self.marker_id_counter = 0  # give each marker a unique idea

    # Save some commenly used variables in the setup class
    self.ref_link = self.move_group.get_pose_reference_frame()
    self.ee_link = self.move_group.get_end_effector_link()
    self.gripper = GripperInterface()

  def remove_all_markers(self):
    """ Utility function to remove all Markers that we potentially published in a previous run of this script. """
    # setup cube / box marker type
    marker = visualization_msgs.msg.Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    # marker.id = 0
    # marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.DELETEALL
    self.marker_publisher.publish(marker)

  def pandaReady(self):
    self.move_group.set_named_target("ready")
    self.move_group.go(wait=True)
    self.gripper.close()

  def attachBox(self, target_name):
    grasping_group = GROUP_NAME_GRIPPER
    touch_links = self.robot.get_link_names(group=grasping_group)
    self.scene.attach_box(self.ee_link, target_name, touch_links=touch_links)

  def detatchBox(self, target_name):
    self.scene.remove_attached_object(self.ee_link, name=target_name)

  def carthesianPath(self,q, pcm=False):
    waypoints = []

    q = quaternion_from_euler(-pi/2, -pi/4, -pi/4)
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = 0.7*np.sqrt(2)/2  # First move up (z)
    wpose.position.y = 0.7*np.sqrt(2)/2 # and sideways (y)
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    waypoints.append(copy.deepcopy(wpose))

    q = quaternion_from_euler(-pi/2, -pi/4, 0)
    wpose.position.x = 0  # Second move forward/backwards in (x)
    wpose.position.y = 0.6
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.z = 0.52  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = 0.44  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))    

    #self.move_group.set_planning_time(10.0)  

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      5.0 )         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    self.move_group.execute(plan, wait=True)
    self.move_group.clear_pose_targets()

    return plan, fraction

  def go_to(self, pos, q, pcm = False):
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    wpose.position.x = pos[0] # First move up (z)
    wpose.position.y = pos[1] # and sideways (y)
    wpose.position.z = pos[2] # and sideways (y)
    wpose.orientation.x = q[0]
    wpose.orientation.y = q[1]
    wpose.orientation.z = q[2]
    wpose.orientation.w = q[3]
    waypoints.append(copy.deepcopy(wpose))

    self.move_group.set_planning_time(10.0)  

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      5.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    self.move_group.execute(plan, wait=True)
    self.move_group.clear_pose_targets()

    return plan, fraction

  def move(self, pos, q, pcm = False):
    #robot = moveit_commander.RobotCommander()
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = q[0]
    pose_goal.orientation.y = q[1]
    pose_goal.orientation.z = q[2]
    pose_goal.orientation.w = q[3]
    pose_goal.position.x = pos[0]
    pose_goal.position.y = pos[1]
    pose_goal.position.z = pos[2]
    self.move_group.set_pose_target(pose_goal)

    if pcm:
      self.move_group.set_path_constraints(pcm)
      self.move_group.set 

    # Calling `stop()` ensures that there is no residual movement
    self.move_group.go(wait=True)
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    self.move_group.clear_path_constraints()

  def pick(self, target_pose, target_name):
    self.pandaReady()

    pos = np.array(target_pose) - np.array([0.15, 0, 0])
    q = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
    self.move(pos, q)
    self.gripper.grasp(width=0.07,force=0)

    target_pose = np.array(target_pose) - np.array([0.02, 0, 0])
    self.go_to(target_pose, q)
    self.gripper.grasp(width=0.0,force=70)

    self.attachBox(target_name)

    #pcm = self.create_line_constraints([0.008, 0.008, 1.0])
    pos = np.array(target_pose) + np.array([0, 0, 0.17])
    self.go_to(pos, q)

    self.move_group.set_support_surface_name("table1")


  def place(self, target_pose, target_name):
    self.move_group.set_support_surface_name("table2")

    q = quaternion_from_euler(-pi/2, -pi/4, 0)
    plan1, fraction = self.carthesianPath(q)

    if fraction < 1:
      print("ERROR : Incomplete Trajectory... REPLANNING \n")
      self.plan(target_pose, target_name)
      return
    self.gripper.grasp(width=0.07,force=0)
    self.detatchBox(target_name)

    pos = [0, 0.5, 0.43]

    plan1, fraction = self.go_to(pos, q)
    if fraction < 1:
      print("ERROR : Can't Return Gripper... REPLANNING \n")
      self.plan(target_pose, target_name)
      return


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


  def create_collision_object(self,shape_type,pos,size,frame_id,op,object_id):
    col = moveit_msgs.msg.CollisionObject()
    col.id = object_id
    col.operation = op
    col.header.frame_id=frame_id

    # create primitive
    primitive = SolidPrimitive()
    primitive.type = shape_type
    primitive.dimensions = size

    # create pose
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pos[0]
    pose.position.y = pos[1]
    pose.position.z = pos[2]
    pose.orientation.x = pose.orientation.y = pose.orientation.z = 0
    pose.orientation.w = 1


    col.primitives = [primitive]
    col.primitive_poses = [pose]

    return col

  def addCollisionObjects(self):
    #Creating Environment
    # ^^^^^^^^^^^^^^^^^^^^
    # Create collision objects.
    self.scene.add_object(self.create_collision_object(
                                1,
                                [0.6, 0, 0.2],   # place
                                [0.2, 0.4, 0.4], # size
                                REFERENCE_FRAME,
                                0,
                                "table1" ))

    self.scene.add_object(self.create_collision_object(
                                1,
                                [0.0, 0.7, 0.2],
                                [0.4, 0.2, 0.4],
                                REFERENCE_FRAME,
                                0,
                                "table2" ))

    self.scene.add_object(self.create_collision_object(
                                1,
                                [0.58, 0.0, 0.85],
                                [0.1, 0.1, 0.1],
                                REFERENCE_FRAME,
                                0,
                                "camera" ))

    # Read the json file to get the items sizes.
    file = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json", "r")
    content = file.read()
    data = json.loads(content)

    for item in data["yolo"]["items"]:
      holder_pos = np.array(item["pos"]) + np.array((0.1, 0.0, 0.0))
      self.scene.add_object(self.create_collision_object(
                                  1,
                                  item["pos"],
                                  data["items_sizes"][item["type"]],
                                  REFERENCE_FRAME,
                                  0,
                                  item["name"]))
      '''
      self.scene.add_object(self.create_collision_object(
                                  1,
                                  holder_pos,
                                  data["items_sizes"]["holder"],
                                  REFERENCE_FRAME,
                                  2,
                                  item["name"]))'''
    
    file.close()

  def removeCollisionObjects(self):
    try:
      self.scene.remove_world_object("table1")
      self.scene.remove_world_object("table2")
      self.scene.remove_world_object("camera")
      self.scene.remove_world_object('cup_00')
      self.scene.remove_world_object('banana_01')
      self.scene.remove_world_object('apple_02')
    except:
      pass

  def plan(self,target_pose, name):
      #Clean the scene and star it again
    self.removeCollisionObjects()
    self.addCollisionObjects()
    self.detatchBox(name)

    rospy.sleep(1.0)
    self.pick(target_pose, name)

    rospy.sleep(1.0)
    self.place(target_pose, name)

def MoveItPanda(target_pose, name):

  for i in range(1):
    panda = PickAndPlacePanda()
    start = time.time()

    # Wait a bit for ROS things to initialize
    panda.plan(target_pose, name)
    print(" PLANNING COMPLETE! \n")
    print(" ========== FINISHED CODE ========== \n")

    end = time.time()

    #with open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/apple_plan.txt", "a") as f:
    #  f.writelines([str(end - start),"\n"])

  roscpp_shutdown()

if __name__ == "__main__":
  roscpp_initialize(sys.argv)
  rospy.init_node("panda_arm_pick_place_python")

  pose_c = (0.5743647543125, -0.147540978, 0.43)
  pose_a = (0.57385245925, -0.0005122950624999999, 0.43)
  pose_b = (0.5723155740625, 0.148565568125, 0.43)

  MoveItPanda(pose_b, "banana_01")