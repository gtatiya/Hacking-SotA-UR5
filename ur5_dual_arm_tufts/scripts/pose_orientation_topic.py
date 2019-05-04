#!/usr/bin/env python
import rospy
import moveit_commander
from geometry_msgs.msg import Pose, Point, PointStamped



class poseData:

	def __init__(self):


		rospy.init_node('pose_orientation', anonymous=True)

		robot = moveit_commander.RobotCommander()
		right_arm_group = moveit_commander.MoveGroupCommander("ur5_arm_right")

		# print robot.get_current_state()		

		right_arm_pose = rospy.Publisher("/ur5_arm_right/Pose", Pose, queue_size=1)
		left_arm_pose = rospy.Publisher("/ur5_arm_left/Pose", Pose, queue_size=1)




		rospy.spin()

		# right_arm_group.set_named_target('right_home_position')
		# right_arm_group.go(wait=True)
		# print("Point 1")

		# right_arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
		#                             -2.366067038969164, -1.571228181260084, -1.0061550793898952])
		# right_arm_group.go(wait=True)
		# print("Point 2")

		# pose = right_arm_group.get_current_pose().pose
		#

		# Need both position and orientation to 


		# pose = geometry_msgs.msg.Pose()

		# pose.position.x = 0.1
		# pose.position.y = 0.1
		# pose.position.z = 0.1
		# pose.orientation.w = 1.0

		# right_arm_group.set_pose_target(pose)
		# right_arm_group.go(wait=True)
		# print("Point 2")


if __name__ == '__main__':
	arm_group = poseData()
