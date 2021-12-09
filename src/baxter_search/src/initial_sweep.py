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

		# subscribe to the camera and keep storing velocities

		# Create a timer object that will sleep long enough to result in
		# a 10Hz publishing rate
		r = rospy.Rate(10) # 10hz

		print("initialized service")
		# print(Rescue)

		rospy.spin()

	def set_turtlebot_pose(self, request):
		# callback from subscription turtlebot pose pub
		# sets a instance variable with the pose, and prevents future overwrites
		if not self.found_turtlebot and self.stopped and self.current_pose:
			print("setting turtlebot pose:", request)
			print("\n\n\n")
			self.turtlebot_pose = self.current_pose
			self.found_turtlebot = True
		return

	def set_goal_pose(self, request):
		# callback from subscription goal cone pose pub
		# sets a instance variable with the pose, and prevents future overwrites
		if not self.found_goal and self.stopped and self.current_pose:
			print("setting goal cup pose:", request)
			print("\n\n\n")
			self.goal_pose = self.current_pose
			self.found_goal = True
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
			# print("\n\nNORM VAL:", norm, "\n\n")

			request.ik_request.pose_stamped.pose.orientation.x = pose[3] / norm
			request.ik_request.pose_stamped.pose.orientation.y = pose[4] / norm
			request.ik_request.pose_stamped.pose.orientation.z = pose[5] / norm
			request.ik_request.pose_stamped.pose.orientation.w = pose[6] / norm
			# request.ik_request.pose_stamped.pose.orientation.x = 0.0
			# request.ik_request.pose_stamped.pose.orientation.y = 1.0
			# request.ik_request.pose_stamped.pose.orientation.z = 0.0
			# request.ik_request.pose_stamped.pose.orientation.w = 0.0

			try:
				# Send the request to the service
				response = self.compute_ik(request)

				# print("\n\n got ik response \n\n")

				# Print the response HERE
				# print(response)
				group = MoveGroupCommander("right_arm")

				print(group.get_end_effector_link())


				# Setting position and orientation target
				group.set_pose_target(request.ik_request.pose_stamped)

				# # TEMP CODE ASHWAT TESTING
				# group.set_planning_time(10);

				# TRY THIS
				# Setting just the position without specifying the orientation
				###group.set_position_target([0.5, 0.5, 0.0])

				# Plan IK and execute
				group.go()

			except rospy.ServiceException, e:
				print "Service call failed: %s"%e

			rospy.sleep(1.0)
			self.current_pose = pose
			self.stopped = True
			rospy.sleep(1.0)

	def sweep(self, request):
		# subscribe to the turtlebot and goal pose publisher
		turtlebot_sub = rospy.Subscriber("/find_turtlebot_pose", PoseStamped, self.set_turtlebot_pose)
		goal_sub = rospy.Subscriber("/find_goal_pose", PoseStamped, self.set_goal_pose)

		# run the sweep
		# pose_1 = [0.782, -0.642, 0.288, 0.046, 0.987, -0.042, 0.146]
		pose_1 = [0.803, -0.640, 0.336, 0.100, 0.975, -0.104, 0.171]
		pose_2 = [0.662, -0.639, 0.182,0.201, 0.972, -0.055, 0.110]
		# pose_3 = [0.684, -0.301, 0.186,-0.063, 0.998, 0.017, 0.001]
		pose_3 = [0.628, -0.159, 0.065, 0.704, -0.690, 0.112, -0.126]
		# pose_4 = [0.623, 0.091, 0.055,-0.273, 0.961, -0.005, 0.042]
		pose_4 = [0.781, 0.150, 0.142, -0.360, 0.929, 0.048, 0.067]

		pose_positions = [pose_1, pose_2, pose_3, pose_4]


		# pose_positions = [[0.013, -0.826, 0.305, -0.018, 1.000, -0.010, 0.009], [0.197, -0.836, 0.361, 0.004, 0.980, -0.048, 0.193],
		# [0.354, -0.797, 0.402, -0.003, 0.959, -0.053, 0.277], [0.568, -0.727, 0.482, -0.011, 0.903, -0.022, 0.430]]
		self.send_movements(pose_positions)

		# move the baxter to turtle pose
		print("found turtlebot", self.turtlebot_pose)
		# self.send_movements([[self.turtlebot_pose.pose.position.x, self.turtlebot_pose.pose.position.y, self.turtlebot_pose.pose.position.z, 
		# 	self.turtlebot_pose.pose.orientation.x, self.turtlebot_pose.pose.orientation.y, self.turtlebot_pose.pose.orientation.z, self.turtlebot_pose.pose.orientation.w]])
		self.send_movements([self.turtlebot_pose])

		print("turtlebot", self.turtlebot_pose)
		print("\n\n\n")
		print("goal", self.goal_pose)
		print("\n\n\n")
		print("current", self.current_pose)

		# return the goal pose of cone
		pose = PoseStamped()
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
