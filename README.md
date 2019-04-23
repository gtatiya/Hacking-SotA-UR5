# Hacking-SotA-UR5

## UR5 Single Arm + RobotiQ 85 gripper

### Gazebo:
`roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true` <br>
`rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/ur5_single_arm.urdf -urdf -x 0 -y 0 -z 1 -model ur5_single_arm`

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch`

### Launch Gazebo and execute trajectories:
`roslaunch ur5_single_arm_manipulation execute_trajectory.launch`

### Execute trajectories (open Gazebo first):
`rosrun ur5_single_arm_manipulation execute_trajectory.py`

## UR5 Single Arm + RobotiQ 85 gripper (Method 2: moveit_simple_grasps)

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo_2.launch`

### Grasp server:
`roslaunch ur5_single_arm_manipulation grasp_generator_server.launch`

### Spawn Table and Block:
`rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_manipulation)/models/2/table.urdf -urdf -x 0.8 -y 0.0 -z 0.55 -model my_object` <br>
`rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_manipulation)/models/2/model.urdf -urdf -x 0.7 -y -0.1 -z 0.6 -model block`

### Pick and Place:
`rosrun ur5_single_arm_manipulation pick_and_place_2.py`

## UR5 Single Arm

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo_arm.launch`

### Launch Gazebo and execute trajectories:
`roslaunch ur5_single_arm_ur5_manipulation execute_trajectory.launch`

### Execute trajectories (open Gazebo first):
`rosrun ur5_single_arm_ur5_manipulation execute_trajectory.py`

## RobotiQ 85 gripper

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo_gripper.launch`

### Launch Gazebo and execute trajectories:
`roslaunch ur5_single_arm_gripper_manipulation execute_trajectory.launch`

### Execute trajectories (open Gazebo first):
`rosrun ur5_single_arm_gripper_manipulation execute_trajectory.py`

