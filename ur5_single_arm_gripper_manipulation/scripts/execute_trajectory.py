#!/usr/bin/env python
import rospy
import moveit_commander

rospy.init_node('execute_trajectory', anonymous=True)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("gripper")

# You can get the reference frame for a certain group by executing this line:
print "Reference frame: %s" % arm_group.get_planning_frame()

# You can get the end-effector link for a certaing group executing this line:
print "End effector: %s" % arm_group.get_end_effector_link()

# You can get a list with all the groups of the robot like this:
print "Robot Groups:"
print robot.get_group_names()

# You can get the current values of the joints like this:
print "Current Joint Values:"
print arm_group.get_current_joint_values()

# You can also get the current Pose of the end-effector of the robot like this:
#print "Current Pose:"
#print arm_group.get_current_pose()

# Finally, you can check the general status of the robot like this:
print "Robot State:"
print robot.get_current_state()


# Open
arm_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
arm_group.go(wait=True)
print("Point 1")

# Close
arm_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
arm_group.go(wait=True)
print("Point 2")
