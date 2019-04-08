#!/usr/bin/env python
import rospy
import moveit_commander

import argparse
import struct
import sys
import copy
import time
import random
import os
import subprocess, signal

import rospkg

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
from std_msgs.msg import (
    Header,
    Empty,
)

#Original starting position:
# def load_gazebo_models(box_no = 4, table_pose=Pose(position=Point(x=1.0, y= 0.0, z=0.0)),
#                        table_reference_frame="world",
#                        block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
#                        block_reference_frame="world")


#Experimenting with:
# def load_gazebo_models(box_no = 4, table_pose=Pose(position=Point(x=1.0, y= 0.0, z=0.0)),
#                        table_reference_frame="world",
#                        block_pose=Pose(position=Point(x=0.3, y= 0.1265, z=0.6709)),
#                        block_reference_frame="world")

      
def load_gazebo_models(box_no = 4, table_pose=Pose(position=Point(x=1.0, y= 0.0, z=0.0)),
                       table_reference_frame="world",
                       block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    print("Loading Gazebo Models")
    script_path = os.path.dirname(os.path.abspath(__file__))
    print(script_path) 
    model_path = script_path[:-8]+"/models/"
    
    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')
    
    # Load Blocks URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
  
   # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF service call failed: {0}".format(e))
   
   # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def load_gazebo_block(box_no = 4, block_pose=Pose(position=Point(x=0.6725, y= 0.1265, z=0.7825)),
                       block_reference_frame="world"):
    # Get Models' Path
    script_path = os.path.dirname(os.path.abspath(__file__)) 
    model_path = script_path[:-8]+"/models/"
     
    # Load Blocks URDF
    block_xml = ''
    block_path = "block/model"+ str(box_no) + ".urdf"
    with open (model_path + block_path, "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
  
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

def delete_gazebo_block():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))


def start_rosbag_recording(filename):
    # # find the directory to save to
    # rospy.loginfo(rospy.get_name() + ' start')
    # script_path = os.path.dirname(os.path.abspath(__file__))
    # rosbagfile_dir = script_path[:-8]+"/rosbagfiles/"
    
    # #  modify the rosbag process with prof Jivko
    # rosbag_process = subprocess.Popen('rosbag record -o {} /robot/joint_states'.format(filename), stdin=subprocess.PIPE, shell=True, cwd= rosbagfile_dir)
    # return rosbag_process

    # Joint efforts seem to not be setup
    pass
    
def stop_rosbag_recording(p):
    # rospy.loginfo(rospy.get_name() + ' stop recording.')
    # rospy.loginfo(p.pid)
    
    # import psutil
    # process = psutil.Process(p.pid)
    # for sub_process in process.children(recursive=True):
    #     sub_process.send_signal(signal.SIGINT)
    # p.wait()  # we wait for children to terminate

    # Joint efforts seem to not be setup
    pass
    
    rospy.loginfo("I'm done")

def addnoise_pose():
    overhead_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)
    #pose = Pose(position= Point(x=0.7, y=0.15, z=-0.129), orientation=overhead_orientation)
    pose = Pose(position= Point(x=-0.3, y=0, z=0.8), orientation=overhead_orientation)
    x = random.uniform(-0.09, 0.09)
    y = random.uniform(-0.09, 0.09)
    pose.position.x = pose.position.x + x
    pose.position.y = pose.position.y + y
    return pose

def move_to_start():
    arm_group.set_named_target('up')
    arm_group.go(wait=True)
    print("Moved to start")
    # Below gives kind of the right overhead orientation
    # arm_group.set_joint_value_target([-0.21957805043352518, -1.097296859939564, 1.8945345194815335,
    #                         -2.366067038969164, -1.571228181260084, -1.0061550793898952])
    # arm_group.go(wait=True)

def gripper_open():
    grp_group.set_joint_value_target([9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05, 9.800441184282249e-05, -9.800441184282249e-05, 9.800441184282249e-05])
    grp_group.go(wait=True)

def gripper_close():
    grp_group.set_joint_value_target([0.8039005131791948, -0.8039005131791948, 0.8039005131791948, 0.8039005131791948, -0.8039005131791948, 0.8039005131791948])
    grp_group.go(wait=True)

def move_to_pose(pose):
    arm_group.set_pose_target(pose)
    arm_group.go(wait=True)

def approach(pose):
    approachPose = copy.deepcopy(pose)
    hover_distance = 0.15
    approachPose.position.z = approachPose.position.z + hover_distance
    arm_group.set_pose_target(approachPose)
    arm_group.go(wait=True)


	
def pick(pose, filename):
    fn = "ur5_grasp_model_" + filename
    filename = "ur5_pick_model_" + filename
    #rps = start_rosbag_recording(fn)
    time.sleep(0.5)
    gripper_open()
    approach(pose)
    move_to_pose(pose)
    gripper_close()
    #stop_rosbag_recording(rps)
    #rosbag_process = start_rosbag_recording(filename)
    time.sleep(0.5)
    approach(pose)
    #stop_rosbag_recording(rosbag_process)


def place(pose, filename):
    fn = "ur5_place_model" + filename
    #rosbag_process = start_rosbag_recording(fn)
    approach(pose)
    move_to_pose(pose)
    gripper_open()
    #stop_rosbag_recording(rosbag_process)
    approach(pose)

robot = moveit_commander.RobotCommander()
arm_group = moveit_commander.MoveGroupCommander("manipulator")
grp_group = moveit_commander.MoveGroupCommander("gripper")

def main():

    rospy.init_node('pick_and_place', anonymous=True)

    # robot = moveit_commander.RobotCommander()
    # arm_group = moveit_commander.MoveGroupCommander("manipulator")
    # grp_group = moveit_commander.MoveGroupCommander("gripper")

    print("Robot is setup")


    # # You can get the reference frame for a certain group by executing this line:
    # print "Arm Reference frame: %s" % arm_group.get_planning_frame()
    # print "Gripper Reference frame: %s" % grp_group.get_planning_frame()

    # # You can get the end-effector link for a certaing group executing this line:
    # print "Arm End effector: %s" % arm_group.get_end_effector_link()
    # print "Gripper End effector: %s" % grp_group.get_end_effector_link()

    # # You can get a list with all the groups of the robot like this:
    # print "Robot Groups:"
    # print robot.get_group_names()

    # # You can get the current values of the joints like this:
    # print "Arm Current Joint Values:"
    # print arm_group.get_current_joint_values()
    # print "Gripper Current Joint Values:"
    # print grp_group.get_current_joint_values()

    # # You can also get the current Pose of the end-effector of the robot like this:
    # print "Arm Current Pose:"
    # print arm_group.get_current_pose()

    # # Finally, you can check the general status of the robot like this:
    # print "Robot State:"
    # print robot.get_current_state()
    
    #parse argument
    #UNCOMMENT BELOW
    #myargv = rospy.myargv(argv=sys.argv)
    filename = str(0)
    num_of_run = 1#int(myargv[1])
    
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    #UNCOMMENT BELOW
    #load_gazebo_models(myargv[1])
    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_gazebo_models)
    
    move_to_start()

    block_pose = Pose(position=Point(x=-0.2, y= 0, z=0.6))

    for iBlock in range(0,12):
        print("Block " + str(iBlock))
        filename = str(iBlock)
        for run in range(0, num_of_run):
            if(not rospy.is_shutdown()):
                print("Picking up block")
                pick(block_pose, filename)
                placePose = addnoise_pose()
                print("Placing block")
                place(placePose, filename)
                gripper_open()
                move_to_start()
                #delete_gazebo_block()
                #load_gazebo_block(filename)
            else:
                break
   
   
    return 0

if __name__ == '__main__':
    sys.exit(main())