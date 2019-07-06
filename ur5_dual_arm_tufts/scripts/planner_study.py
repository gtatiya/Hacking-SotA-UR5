#!/usr/bin/env python
import rospy
import moveit_commander
import tf
import rospkg
import os
import numpy
import time

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

rospy.init_node('planners', anonymous=True)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")
scene = moveit_commander.PlanningSceneInterface()

planner_time_array = []

planners = [ 
    "RRTConnect"
    ,"RRT"
    ,"SBL"
    ,"EST"
    ,"LBKPIECE"
    ,"BKPIECE"
    ,"KPIECE"
    ,"RRTstar"
    ,"TRRT"
    ,"PRM"
    ,"PRMstar"
    ,"FMT"
    ,"BFMT"
    ,"PDST"
    ,"STRIDE"
    ,"BiTRRT"
    ,"LBTRRT"
    ,"BiEST"
    ,"ProjEST"
    ,"LazyPRM"
    ,"LazyPRMstar"
    ,"SPARS"
    ,"SPARStwo"]



pose = arm_group.get_current_pose().pose


scene.remove_world_object("obstacle")

# Start point
pose.position.x = 0.4
pose.position.y = -0.2
pose.position.z = 1

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
# print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

pose_A = pose

arm_group.set_pose_target(pose_A)
time.sleep(2)
arm_group.go(wait=True)
print("Start point")

# End point
pose.position.x = 0.4
pose.position.y = 0.2
pose.position.z = 1

downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
# print("downOrientation: ", downOrientation)
pose.orientation.x = downOrientation[0]
pose.orientation.y = downOrientation[1]
pose.orientation.z = downOrientation[2]
pose.orientation.w = downOrientation[3]

pose_B = pose

raw_input("Press Enter to continue...")

for i in range(len(planners)):
  arm_group.set_planner_id(planners[i])
  print("Current planner:", planners[i])
  arm_group.set_pose_target(pose_B)

  try:
    plan = arm_group.plan()
    time_from_start = plan.joint_trajectory.points[-1].time_from_start.secs + plan.joint_trajectory.points[-1].time_from_start.nsecs / 100000000.0

    print("Planner Time to point B: ", time_from_start)
    start_time = time.time()
    time.sleep(2)
    arm_group.go(wait=True)    
    print("Pose_B")
    elapsed_time = time.time() - start_time
    print("Elapsed time: " + str(elapsed_time))

    planner_time_array.append([planners[i], time_from_start, elapsed_time])

    # return to point A

    time.sleep(2)
    arm_group.set_pose_target(pose_A)
    time.sleep(2)
    arm_group.go(wait=True)
    print("Start point")

  except IndexError:
    pass






print(planner_time_array)

# with open('/home/drevinci/catkin_ws/src/Hacking-SotA-UR5/ur5_dual_arm_tufts/planner_logs.csv', mode='w') as csv_file:
#     logger = csv.writer(csv_file, delimiter=',')

#     for i in range(len(planner_time_array)):
#         logger.writerow()


# Adding obstacle
# p = PoseStamped()
# p.header.frame_id = robot.get_planning_frame()
# p.header.stamp = rospy.Time.now()

# p.pose.position.x = 0.85
# p.pose.position.y = 0.0
# p.pose.position.z = 0.70

# q = tf.transformations.quaternion_from_euler(0.0, 0.0, numpy.deg2rad(90.0))
# p.pose.orientation = Quaternion(*q)

# print("Adding obstacle...")
# scene.add_box("obstacle", p, (0.05, 1, 1))
# scene.add_box("obstacle", p, (0.05, 1, 1))


# # Start point
# pose.position.x = 0.4
# pose.position.y = -0.2
# pose.position.z = 1

# downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
# # print("downOrientation: ", downOrientation)
# pose.orientation.x = downOrientation[0]
# pose.orientation.y = downOrientation[1]
# pose.orientation.z = downOrientation[2]
# pose.orientation.w = downOrientation[3]

# arm_group.set_pose_target(pose_A)
# planner_time_A = arm_group.get_planning_time()
# print("Plan Time A", planner_time_A)
# arm_group.go(wait=True)
# print("Pose_A")

# # End point
# pose.position.x = 0.4
# pose.position.y = 0.2
# pose.position.z = 1

# downOrientation = tf.transformations.quaternion_from_euler(0, 3.1415/2, 0)
# # print("downOrientation: ", downOrientation)
# pose.orientation.x = downOrientation[0]
# pose.orientation.y = downOrientation[1]
# pose.orientation.z = downOrientation[2]
# pose.orientation.w = downOrientation[3]

# arm_group.set_pose_target(pose_B)
# arm_group.go(wait=True)
# print("End point")

# scene.remove_world_object("obstacle")
