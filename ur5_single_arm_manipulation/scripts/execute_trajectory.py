#!/usr/bin/env python
import rospy
import moveit_commander

rospy.init_node('execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")

# You can get the reference frame for a certain group by executing this line:
print "Arm Reference frame: %s" % arm_group.get_planning_frame()
print "Gripper Reference frame: %s" % grp_group.get_planning_frame()

# You can get the end-effector link for a certaing group executing this line:
print "Arm End effector: %s" % arm_group.get_end_effector_link()
print "Gripper End effector: %s" % grp_group.get_end_effector_link()

# You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()

# You can get the current values of the joints like this:
print "Arm Current Joint Values:"
print arm_group.get_current_joint_values()
print "Gripper Current Joint Values:"
print grp_group.get_current_joint_values()

# You can also get the current Pose of the end-effector of the robot like this:
print "Arm Current Pose:"
print arm_group.get_current_pose()

# Finally, you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()


arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 1")

arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
                            -2.366067038969164, -1.571228181260084, -1.0061550793898952])
arm_group.go(wait=True)
print("Point 2")

pose = arm_group.get_current_pose().pose
print("Got pose 1")
print(pose)

# Close
grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
grp_group.go(wait=True)
print("Point 3")

pose = arm_group.get_current_pose().pose
print("Got pose 2")
print(pose)
# pose.position.x += 0.1
# pose.position.y += 0.1
# pose.position.z += 0.1
########################
# pose.position.x -= 0.5
# pose.position.y += 0.1
# pose.position.z += 0.8
# pose.position.x += 0.3
# pose.position.y += 0.3
# pose.position.z += 0.3

print("Got pose 4")
print(pose)

# pose.position.x = -0.355057120323
# pose.position.y = -0.192236363888
# pose.position.z = 0.532070159912

pose.position.x = -0.7
pose.position.y = 0.2
pose.position.z = 1.5

"""
  x: -0.32523688674
  y: 0.154882699251
	z: 0.560538291931

	Up location:
  x: -0.726045787334
  y: 0.206279024482
 z: 1.50351989269
  x: -0.729166285085
  y: 0.191447970874
  z: 1.508802245

Try diff planner
Try IKFast: http://openrave.org/docs/0.8.2/openravepy/ikfast/

"""
arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 4")

# Open
grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
grp_group.go(wait=True)
print("Point 5")

pose = arm_group.get_current_pose().pose
print("Got pose 5")
print(pose)

# arm_group.set_named_target('up')
# arm_group.go(wait=True)
# print("Point 6")
