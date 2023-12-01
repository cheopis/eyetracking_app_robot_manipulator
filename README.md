# Eyetracking App for Robot Arm Manipulation

instalação:
- ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu
- RTGene: https://github.com/Tobias-Fischer/rt_gene
- moveit: sudo apt install ros-melodic-moveit
- panda Melodic: https://github.com/justagist/panda_simulator (seguir tutorial melodic-devel)
[pip install numpy-quaternion future argparse]
- Darknet/YOLO : https://github.com/leggedrobotics/darknet_ros

Rodar gazebo:
 roslaunch panda_gazebo panda_world.launch start_moveit:=false
 roslaunch panda_sim_moveit sim_move_group.launch
 
Rodar YOLO:
 roslaunch darknet_ros darknet_ros.launch image:=/camera/image_raw

