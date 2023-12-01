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
    roscpp_initialize(sys.argv)
    rospy.init_node("panda_arm_pick_place_python")
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

  def create_plane_constraints(self, size = [2.0, 2.0, 0.1]):
    pcm = moveit_msgs.msg.PositionConstraint()
    pcm.header.frame_id = self.ref_link
    pcm.link_name = self.ee_link

    cbox = SolidPrimitive()
    cbox.type = SolidPrimitive.BOX
    cbox.dimensions = size
    pcm.constraint_region.primitives.append(cbox)

    current_pose = self.move_group.get_current_pose()

    cbox_pose = geometry_msgs.msg.Pose()
    cbox_pose.position.x = current_pose.pose.position.x
    cbox_pose.position.y = current_pose.pose.position.y
    cbox_pose.position.z = current_pose.pose.position.z

    # turn the constraint region 45 degrees around the x-axis.
    quat = quaternion_from_euler(0, 0, 0)
    cbox_pose.orientation.x = quat[0]
    cbox_pose.orientation.y = quat[1]
    cbox_pose.orientation.z = quat[2]
    cbox_pose.orientation.w = quat[3]
    pcm.constraint_region.primitive_poses.append(cbox_pose)
    pcm.weight = 1.0

    # display the constraints in rviz
    self.display_box(cbox_pose, cbox.dimensions)

    return pcm

  ## Building on the previous constraint, we can also make it a line, by also reducing the dimension of th box in the x-direction.
  def create_line_constraints(self, pos):
    pcm = moveit_msgs.msg.PositionConstraint()
    pcm.header.frame_id = self.ref_link
    pcm.link_name = self.ee_link

    cbox = SolidPrimitive()
    cbox.type = SolidPrimitive.BOX
    cbox.dimensions = pos
    pcm.constraint_region.primitives.append(cbox)

    current_pose = self.move_group.get_current_pose()

    cbox_pose = geometry_msgs.msg.Pose()
    cbox_pose.position.x = current_pose.pose.position.x
    cbox_pose.position.y = current_pose.pose.position.y
    cbox_pose.position.z = current_pose.pose.position.z
    quat = quaternion_from_euler(0, 0, 0)
    cbox_pose.orientation.x = quat[0]
    cbox_pose.orientation.y = quat[1]
    cbox_pose.orientation.z = quat[2]
    cbox_pose.orientation.w = quat[3]
    pcm.constraint_region.primitive_poses.append(cbox_pose)
    pcm.weight = 1.0
    
    # display the constraints in rviz
    self.display_box(cbox_pose, cbox.dimensions)



    return pcm

  def display_box(self, pose, dimensions):
    """ Utility function to visualize position constraints. """
    assert len(dimensions) == 3

    # setup cube / box marker type
    marker = visualization_msgs.msg.Marker()
    marker.header.stamp = rospy.Time.now()
    marker.ns = "/"
    marker.id = self.marker_id_counter
    marker.type = visualization_msgs.msg.Marker.CUBE
    marker.action = visualization_msgs.msg.Marker.ADD
    marker.color = COLOR_TRANSLUCENT
    marker.header.frame_id = self.ref_link

    # fill in user input
    marker.pose = pose
    marker.scale.x = dimensions[0]
    marker.scale.y = dimensions[1]
    marker.scale.z = dimensions[2]

    # publish it!
    self.marker_publisher.publish(marker)
    self.marker_id_counter += 1

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

  def go_to(self, pos, q, pcm=False):
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
      path_constraints = moveit_msgs.msg.Constraints()
      path_constraints.position_constraints.append(pcm)
      path_constraints.name = "use_equality_constraints"
      self.move_group.set_path_constraints(path_constraints)

    self.move_group.set_planning_time(10.0)
    self.move_group.plan()
    self.move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    self.move_group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    self.move_group.clear_pose_targets()
    self.move_group.clear_path_constraints()

  def pick(self, target_pose, target_name):
    self.pandaReady()

    pos = np.array(target_pose) - np.array([0.1, 0, 0])
    q = quaternion_from_euler(-pi/2, -pi/4, -pi/2)
    self.go_to(pos, q)
    self.gripper.grasp(width=0.07,force=0)

    target_pose = np.array(target_pose) - np.array([0.01, 0, 0])
    self.go_to(target_pose,q)
    self.gripper.grasp(width=0.0,force=70)

    self.attachBox(target_name)

    pcm = self.create_line_constraints([0.008, 0.008, 1.0])
    pos = np.array(target_pose) + np.array([0, 0, 0.22])
    self.go_to(pos, q, pcm)

    self.move_group.set_support_surface_name("table1")

  def place(self, target_name):
    self.move_group.set_support_surface_name("table2")

    q = quaternion_from_euler(-pi/2, -pi/4, 0)
    pos =[0.0, 0.6, 0.65]
    pcm = self.create_plane_constraints()
    self.go_to(pos, q, pcm)

    pos = [0.0, 0.6, 0.44]
    pcm = self.create_line_constraints([0.008, 0.008, 1.0])
    self.go_to(pos, q, pcm)

    self.gripper.grasp(width=0.07,force=0)
    self.detatchBox(target_name)

    pos = [0, 0.5, 0.44]
    pcm = self.create_line_constraints([0.008, 1.0, 0.008])
    self.go_to(pos, q, pcm)

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

    # Read the json file to get the items sizes.
    file = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/data.json", "r")
    content = file.read()
    data = json.loads(content)

    for item in data["yolo"]["items"]:
      holder_pos = np.array(item["pos"]) + np.array((0.0, 0.0, 0.02))
      self.scene.add_object(self.create_collision_object(
                                  1,
                                  item["pos"],
                                  data["items_sizes"][item["type"]],
                                  REFERENCE_FRAME,
                                  0,
                                  item["name"]))
      
      self.scene.add_object(self.create_collision_object(
                                  1,
                                  holder_pos,
                                  data["items_sizes"]["holder"],
                                  REFERENCE_FRAME,
                                  2,
                                  'holder'))
    
    file.close()

  def removeCollisionObjects(self):
    try:
      self.scene.remove_world_object("table1")
      self.scene.remove_world_object("table2")
      self.scene.remove_world_object('apple_01')
    except:
      pass

def MoveItPanda(target_pose, name):
  panda = PickAndPlacePanda()
  start = time.time()

  #Clean the scene and star it again
  panda.removeCollisionObjects()
  panda.addCollisionObjects()
  
  # Wait a bit for ROS things to initialize
  rospy.sleep(1.0)
  panda.pick(target_pose, name)

  rospy.sleep(1.0)
  panda.place(name)

  end = time.time()
  '''
  item = show_gazebo_models('box_apple')[0]
  end_position = np.array([item.x,item.y,item.z])
  f = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/hybridmoveit_data.txt", "a")
  f.writelines([str(end - start),";", str(end_position),"\n"])
  '''
  f = open("/home/ariele/catkin_ws/src/gazebo_eyegaze/src/app/hybridmoveit_data.txt", "a")
  f.writelines([str(end - start),"\n"])
  roscpp_shutdown()

if __name__ == "__main__":
  roscpp_initialize(sys.argv)

  pose = (0.4855737699, 0.0016393442, 0.43)

  MoveItPanda(pose, "apple_01")