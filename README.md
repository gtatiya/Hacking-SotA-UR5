# Hacking-SotA-UR5

## UR5 Single Arm + RobotiQ 85 gripper

### Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_rviz.launch`

### Gazebo:
```
roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true
rosrun gazebo_ros spawn_model -file $(rospack find ur5_single_arm_tufts)/urdf/ur5_single_arm.urdf -urdf -x 0 -y 0 -z 0 -model ur5_single_arm
```

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_tufts ur5_single_arm_gazebo.launch`

### Launch Gazebo and execute trajectories:
`roslaunch ur5_single_arm_manipulation execute_trajectory.launch`

### Execute trajectories (open Gazebo first):
`rosrun ur5_single_arm_manipulation execute_trajectory.py`

## Pick and Place (Method: moveit_simple_grasps)

<img src="pics/pick_and_place_demo.gif" align="middle">

### Gazebo + MoveIt Rviz:
`roslaunch ur5_single_arm_manipulation pick_and_place.launch`

### Grasp server:
`roslaunch ur5_single_arm_manipulation grasp_generator_server.launch`

### Pick and Place:
`rosrun ur5_single_arm_manipulation pick_and_place.py`

## Obstacle Avoidance
```
roslaunch ur5_single_arm_manipulation planners.launch
rosrun ur5_single_arm_manipulation planners.py
```

## UR5 Dual Arm + RobotiQ 85 gripper

roslaunch ur5_dual_arm_tufts ur5_dual_arm_gazebo.launch

### UR5 Right Arm + RobotiQ 85 gripper

roslaunch ur5_dual_arm_tufts ur5_right_arm_gazebo.launch

### UR5 Left Arm + RobotiQ 85 gripper

roslaunch ur5_dual_arm_tufts ur5_left_arm_gazebo.launch

