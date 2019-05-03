#!/usr/bin/env python
import rospy
import moveit_commander
import geometry_msgs.msg

rospy.init_node('pose_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
right_arm_group = moveit_commander.MoveGroupCommander("ur5_arm_right")

# You can get the reference frame for a certain group by executing this line:
print "Reference frame: %s" % right_arm_group.get_planning_frame()

# You can get the end-effector link for a certaing group executing this line:
print "End effector: %s" % right_arm_group.get_end_effector_link()

# You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()

# You can get the current values of the joints like this:
print "Current Joint Values:"
print right_arm_group.get_current_joint_values()

# You can also get the current Pose of the end-effector of the robot like this:
print "Current Pose:"
print right_arm_group.get_current_pose()

# Finally, you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()


right_arm_group.set_named_target('right_home_position')
right_arm_group.go(wait=True)
print("Point 1")

# right_arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
#                             -2.366067038969164, -1.571228181260084, -1.0061550793898952])
# right_arm_group.go(wait=True)
# print("Point 2")

# pose = right_arm_group.get_current_pose().pose
#

# Need both position and orientation to 


pose = geometry_msgs.msg.Pose()

pose.position.x = 0.1
pose.position.y = 0.1
pose.position.z = 0.1
pose.orientation.w = 1.0

right_arm_group.set_pose_target(pose)
right_arm_group.go(wait=True)
print("Point 2")
