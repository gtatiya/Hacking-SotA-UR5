#!/usr/bin/env python
import rospy
import moveit_commander
import tf

rospy.init_node('execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")

# arm_group.set_planner_id("LBKPIECE")

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

"""
The Lenght of gripper is ~0.15
Min. reachable z value on the table is 0.9 from downOrientation

Right front corner of table
  x: 1.22034440041
  y: 0.587998211384
  z: 0.736139118671

Right back corner of table
  x: 0.44
  y: 0.55
  z: 0.720291554928

Left front corner of table
  x: 1.2339609623
  y: -0.448462069035
  z: 0.738923251629

Left back corner of table
  x: 0.450013375282
  y: -0.476064682007
  z: 0.73510235548
"""

arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 1")

pose = arm_group.get_current_pose().pose
# Max x points
# pose.position.x = 0.90 #0.45, 0.55, 0.65, 0.75, 0.85, 0.90
# pose.position.y = 0.3 #-0.1, 0.0, 0.1
# pose.position.z = 0.9

# Min x points
# pose.position.x = 0.45 #0.45, 0.55, 0.65, 0.75, 0.85, 0.90
# pose.position.y = 0.1 #-0.45, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1
# pose.position.z = 0.9

# Block point
pose.position.x = 0.50
pose.position.y = 0.0
pose.position.z = 0.9

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 2")


# arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
#                             -2.366067038969164, -1.571228181260084, -1.0061550793898952])
# arm_group.go(wait=True)
# print("Point 2")

# Close
grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
grp_group.go(wait=True)
print("Point 3")

pose = arm_group.get_current_pose().pose
pose.position.x += 0.1
pose.position.y += 0
pose.position.z += 0
arm_group.set_pose_target(pose)
arm_group.go(wait=True)
print("Point 4")

# Open
grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
grp_group.go(wait=True)
print("Point 5")

arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 6")
