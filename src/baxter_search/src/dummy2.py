#!/usr/bin/python

import rospy
from baxter_search.srv import Guide, SweepPose
from geometry_msgs.msg import PoseStamped
from turtlebot_nav.srv import Rescue

if __name__ == '__main__':
	rospy.init_node('dummy')
	
	# rospy.wait_for_service('/baxter_initial_sweep')
	# sweep_srv = rospy.ServiceProxy('/baxter_initial_sweep', SweepPose)

	rospy.wait_for_service('/guide')
	guide_srv = rospy.ServiceProxy('/guide', Guide)


	rospy.wait_for_service('/turtlebot_last_leg')
	rescue_srv = rospy.ServiceProxy('/turtlebot_last_leg', Rescue)


	# x = 0.619
	# y = 0.163
	# z = 0.093
	# goal = PoseStamped()
	# goal.header.frame_id = "base"

	# #x, y, and z position
	# goal.pose.position.x = x
	# goal.pose.position.y = y
	# goal.pose.position.z = z

	# #Orientation as a quaternion
	# goal.pose.orientation.x = 0.864
	# goal.pose.orientation.y = -0.464
	# goal.pose.orientation.z = 0.057
	# goal.pose.orientation.w = -0.188

	# --------------------------------- liked

	x = 0.628
	y = -0.159
	z = 0.065
	goal = PoseStamped()
	goal.header.frame_id = "base"

	#x, y, and z position
	goal.pose.position.x = x
	goal.pose.position.y = y
	goal.pose.position.z = z

	#Orientation as a quaternion
	goal.pose.orientation.x = 0.704
	goal.pose.orientation.y = -0.690
	goal.pose.orientation.z = 0.112
	goal.pose.orientation.w = -0.126

	# -----------------------------------


	# x = 0.697
	# y = -0.045
	# z = 0.088
	# goal = PoseStamped()
	# goal.header.frame_id = "base"

	# #x, y, and z position
	# goal.pose.position.x = x
	# goal.pose.position.y = y
	# goal.pose.position.z = z

	# #Orientation as a quaternion
	# goal.pose.orientation.x = 0.848
	# goal.pose.orientation.y = -0.514
	# goal.pose.orientation.z = 0.079
	# goal.pose.orientation.w = -0.098
	
	while not rospy.is_shutdown():
		r = raw_input("Enter to begin")
		guide_srv(goal, [5, 0])
		rescue_srv()