# Hacking-SotA-UR5

## UR5 Single Arm + RobotiQ 85 gripper

## Gazebo
`roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true` 
`rosrun gazebo_ros spawn_model -file src/ur5_single_arm_tufts/urdf/ur5_single_arm.urdf -urdf -x 0 -y 0 -z 1 -model ur5_single_arm`

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch`

### Launch Gazebo and execute trajectories:
`roslaunch ur5_single_arm_manipulation execute_trajectory.launch`

### Execute trajectories (open Gazebo first):
`rosrun ur5_single_arm_ur5_manipulation execute_trajectory.py`

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

