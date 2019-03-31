# Hacking-SotA-UR5

## Gazebo
`roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true` 

`rosrun gazebo_ros spawn_model -file src/ur5_single_arm_tufts/urdf/ur5_single_arm.urdf -urdf -x 0 -y 0 -z 1 -model ur5_single_arm`
