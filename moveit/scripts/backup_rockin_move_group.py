#!/usr/bin/env python
import roslib
roslib.load_manifest('mir_moveit_youbot_brsu_2')
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

#joint known positions
candle = [2.9496, 1.13446, -2.54818, 1.78896, 2.93075]
out_of_view = [2.94958, 0.01564, -2.59489, 2.38586, 2.93068]
pre_grasp = [3.02221, 2.48996, -1.53309, 1.17502, 2.92980]
grasp_standing = [2.93836, 2.020597, -1.88253, 3.36243, 3.01283]
tower_right = [2.5061, 0.0935881, -2.60509, 1.42038, 2.93033]
platform_right = [2.71339, 0.156002, -3.15581, 1.04624, 3.09898]

#global variable declaration:
#Instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()
#Instantiate a PlanningSceneInterface object
scene = moveit_commander.PlanningSceneInterface()
#Instantiate a MoveGroupCommander object
group = moveit_commander.MoveGroupCommander("arm_1")
#create this DisplayTrajectory publisher which is used below to  publish trajectories for RVIZ to visualize.
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory)

def move_arm_joint_space(target_joint_angles):
	print "=== PLANNING IN JOINT SPACE ==="
	group.clear_pose_targets()
	print "Current joint values: %s" % group.get_current_joint_values()
	print "Target angles : ", target_joint_angles
	group.set_joint_value_target(target_joint_angles)
	try:
		plan = group.plan()
		rospy.sleep(1)
		group.execute(plan)
		rospy.sleep(5) 
	except rospy.ROSInterruptException:
		pass

def cartesian_path(desired_path):
	print "=== CARTESIAN PATH ==="
	print "Path to execute : "
	pose_i = 0
	for poses in desired_path:
		print "pose [", pose_i, "] : ", poses
		pose_i+=1
	try:
		#interpolate at a resolution of 1cm (0.01) 
		(plan2, fraction) = group.compute_cartesian_path(
                             desired_path,   # waypoints to follow
                             0.01,        # eef_step
                             0.0)         # jump_threshold
		rospy.sleep(1)
		print "Executing plan now..."
		group.execute(plan2)
	except rospy.ROSInterruptException:
		pass

def node_callback(msg): #callback for the /goal_end_effector_pose topic
	print "Received goal position : ", msg.pose.position
	print "Received goal orientation : ", msg.pose.orientation
	print "Goal orientation is not implemented... we will go to goal position"
	received_cart_pose = geometry_msgs.msg.Pose()
	print "Going to pregrasp position"
	move_arm_to_predefined_pose("grasp_standing", grasp_standing)
	print "Going to received pose as published on topic : /goal_end_effector_pose"
	move_arm_to_cartesian_pose(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
	spinning_message()

def move_arm_to_predefined_pose(position, angles):
	print "Moving arm to ", position ,"..."
	move_arm_joint_space(angles)

def move_arm_to_cartesian_pose(x, y, z):
	waypoints = []
	waypoints.append(group.get_current_pose().pose)
	#Orient gripper
	wpose = geometry_msgs.msg.Pose()
	wpose.position.x = waypoints[0].position.x
	wpose.position.y = waypoints[0].position.y
	wpose.position.z = waypoints[0].position.z
	wpose.orientation.w = 1.0
	wpose.position.x = x - 0.03 #substracting offset between link 5 and gripper
	wpose.position.y = y
	wpose.position.z = z + 0.015 #test
	waypoints.append(copy.deepcopy(wpose))
	cartesian_path(waypoints)

def test_this_node():
	move_arm_to_predefined_pose("candle", candle)
	move_arm_to_predefined_pose("grasp_standing", grasp_standing)
	#pregrasp standing cart position = 0.458, 0.008, 0.130
	move_arm_to_cartesian_pose(0.5, 0.0, 0.3)

def spinning_message():
	print "ros will spin now... waiting for /goal_end_effector_pose topic"

if __name__=='__main__':
	#initialize moveit_commander and rospy
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('moveit_pose_listener', anonymous=True)
	rospy.loginfo("move_group running")
	rospy.Subscriber("/goal_end_effector_pose", PoseStamped, node_callback)
	print "Planning reference frame : " , group.get_planning_frame()
	print "End effector reference frame : " , group.get_end_effector_link()
	#test_this_node()
	spinning_message()
	rospy.spin()