#!/usr/bin/python

import rospy
from baxter_search.srv import Guide
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
	rospy.init_node('dummy')
	rospy.wait_for_service('/guide')
	guide_srv = rospy.ServiceProxy('/guide', Guide)

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
	
	g = Guide()
	g.goal_pose = goal
	g.ignore_tags = [5, 0]
	while not rospy.is_shutdown():
		r = raw_input("Enter to begin")
		guide_srv(goal, [5, 0])
