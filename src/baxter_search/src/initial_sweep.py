#!/usr/bin/python

from baxter_search.srv import SweepPose

import tf2_ros
import rospy

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
from geometry_msgs.msg import Twist

from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import sys

class BaxterInitialSweep:
	def __init__(self):
		#Create a service that does the baxter's initial sweep and returns poses of goal and turtlebot
		rospy.Service('baxter_initial_sweep', SweepPose, self.sweep)

		self.turtlebot_pose = None
		self.goal_pose = None

		self.found_goal = False
		self.found_turtlebot = False

		self.stopped = True

		rospy.wait_for_service('compute_ik')
		arm = 'right'
		# Create the function used to call the service
		self.compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)

		# Create a timer object that will sleep long enough to result in
		# a 10Hz publishing rate
		r = rospy.Rate(10) # 10hz

		print("initialized service")

		rospy.spin()

	def set_turtlebot_pose(self, request):
		# callback from subscription turtlebot pose pub
		# sets a instance variable with the pose, and prevents future overwrites
		if not self.turtlebot_pose and self.stopped and self.current_pose:
			print("setting turtlebot pose:", request)
			print("\n\n\n")
			self.turtlebot_pose = self.current_pose
		return

	def set_goal_pose(self, request):
		# callback from subscription goal cone pose pub
		# sets a instance variable with the pose, and prevents future overwrites
		if not self.goal_pose and self.stopped and self.current_pose:
			print("setting goal cup pose:", request)
			print("\n\n\n")
			self.goal_pose = self.current_pose
		return

	def send_movements(self, pose_positions):
		for i, pose in enumerate(pose_positions):
			self.stopped = False
			# Construct the request
			request = GetPositionIKRequest()
			request.ik_request.group_name = "right_arm"

			link = "right_hand_camera"

			request.ik_request.ik_link_name = link
			request.ik_request.attempts = 20
			request.ik_request.pose_stamped.header.frame_id = "base"

			# Set the desired orientation for the end effector HERE
			request.ik_request.pose_stamped.pose.position.x = pose[0]
			request.ik_request.pose_stamped.pose.position.y = pose[1]
			request.ik_request.pose_stamped.pose.position.z = pose[2]      

			norm = np.sqrt(sum([p**2 for p in pose[3:]]))
			request.ik_request.pose_stamped.pose.orientation.x = pose[3] / norm
			request.ik_request.pose_stamped.pose.orientation.y = pose[4] / norm
			request.ik_request.pose_stamped.pose.orientation.z = pose[5] / norm
			request.ik_request.pose_stamped.pose.orientation.w = pose[6] / norm

			try:
				# Send the request to the service
				response = self.compute_ik(request)
				group = MoveGroupCommander("right_arm")

				group.set_end_effector_link('right_hand_camera')
				print(group.get_end_effector_link())


				# Setting position and orientation target
				group.set_pose_target(request.ik_request.pose_stamped)

				# Plan IK and execute
				group.go()

			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			rospy.sleep(1.0)
			self.current_pose = pose
			self.stopped = True
			rospy.sleep(1.0)

	def sweep(self, request):
		self.turtlebot_pose = None
		self.goal_pose = None
		# subscribe to the turtlebot and goal pose publisher
		turtlebot_sub = rospy.Subscriber("/find_turtlebot_pose", PoseStamped, self.set_turtlebot_pose)
		goal_sub = rospy.Subscriber("/find_goal_pose", PoseStamped, self.set_goal_pose)

		# run the sweep
		pose_1 = [0.818, -0.789, 0.154, -0.549, 0.775, -0.298, 0.097]
		pose_2 = [0.588, -0.627, 0.177, -0.517, 0.843, -0.145, 0.012]
		pose_3 = [0.628, -0.159, 0.065, 0.704, -0.690, 0.112, -0.126]
		pose_4 = [0.799, 0.093, 0.282, 0.965, -0.136, 0.109, -0.195]
		pose_positions = [pose_1, pose_2, pose_3, pose_4]

		self.send_movements(pose_positions)

		# move the baxter to turtle pose
		print("found turtlebot", self.turtlebot_pose)
		self.send_movements([self.turtlebot_pose])

		print("turtlebot", self.turtlebot_pose)
		print("\n\n\n")
		print("goal", self.goal_pose)
		print("\n\n\n")
		print("current", self.current_pose)

		# return the goal pose of cone
		pose = PoseStamped()
		pose.header.frame_id = 'base'
		pose.pose.position.y = self.goal_pose[1]
		pose.pose.position.z = self.goal_pose[2]
		pose.pose.position.x = self.goal_pose[0]
		pose.pose.orientation.x = self.goal_pose[3]
		pose.pose.orientation.y = self.goal_pose[4]
		pose.pose.orientation.z = self.goal_pose[5]
		pose.pose.orientation.w = self.goal_pose[6]

		return pose

if __name__ == '__main__':
 	rospy.init_node('baxter_initial_sweep', anonymous=True)

	controller = BaxterInitialSweep()
