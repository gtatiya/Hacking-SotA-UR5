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
Min. reachable z value on the table is 0.89 from downOrientation
Min. reachable x on table: 0.37

Max. reachable y on table: 0.46

Right front corner of table
  x: 1.3552107811
  y: 0.493778288364
  z: 0.719269096851

Right back corner of table
  x: 0.352727115154
  y: 0.504722833633
  z: 0.714043438435

Left front corner of table
  x: 1.35521054268
  y: -0.491892397404
  z: 0.717385590076

Left back corner of table
  x: 0.363544255495
  y: -0.502857685089
  z: 0.721333742142
"""

arm_group.set_named_target('up')
arm_group.go(wait=True)
print("Point 1")

pose = arm_group.get_current_pose().pose
# Max x points
# pose.position.x = 0.9 #0.9
# pose.position.y = -0.2 # -0.1 to 0.1
# pose.position.z = 0.89

# Min x points
# pose.position.x = 0.37 #0.37
# pose.position.y = -0.46 # -0.46 to 0.46
# pose.position.z = 0.89

# Block point
pose.position.x = 0.4
pose.position.y = 0.0
pose.position.z = 0.89

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
