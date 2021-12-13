#!/usr/bin/python

import rospy
from turtlebot_nav.srv import Rescue
from baxter_search.srv import Guide, SweepPose
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
	rospy.init_node('dummy')
	
	rospy.wait_for_service('/baxter_initial_sweep')
	sweep_srv = rospy.ServiceProxy('/baxter_initial_sweep', SweepPose)

	rospy.wait_for_service('/guide')
	guide_srv = rospy.ServiceProxy('/guide', Guide)

	rospy.wait_for_service('/turtlebot_last_leg')
	rescue_srv = rospy.ServiceProxy('/turtlebot_last_leg', Rescue)

	
	while not rospy.is_shutdown():
		r = raw_input("Enter to begin")
		goal_sweep = sweep_srv()
		print(goal_sweep.goal_pose)
		guide_srv(goal_sweep.goal_pose, [5, 0])
		rescue_srv()
