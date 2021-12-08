#!/usr/bin/python

import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
from ar_track_alvar_msgs.msg import AlvarMarkers
# imports for CV code
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
from geometry_msgs.msg import Twist


class FindTurtlebotAndGoal:
	def __init__(self):
		self.pub_turtlebot_pose = rospy.Publisher("/find_turtlebot_pose", PoseStamped, queue_size=10)
		self.pub_goal_pose = rospy.Publisher("/find_goal_pose", PoseStamped, queue_size=10)
		self.turtlebot_seen = False
		self.goal_seen = False
		self.group = MoveGroupCommander("right_arm")

		# subscriber with callback
		rospy.Subscriber("some_cam_topic", Image, self.check_goal)
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.check_turtlebot)

		rospy.spin()

	def get_current_pose(self):
		return self.group.get_current_pose("right_hand_camera")


	### DETECT IF SEES TURTLEBOT TO BEGIN AT
	def check_turtlebot(self, msg):	
		current = set([m.id for m in msg.markers])
		if 5 in current:
			self.turtlebot_seen = True
			self.pub_turtlebot_pose.publish(self.get_current_pose())

	### DETECT IF SEES GOAL CUPS TO RESCUE
	def check_goal(self, message):
		bridge = CvBridge()
		cv_img = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')

		hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

		# set the lower and upper bounds for the green hue
		lower_green = np.array([50,100,50])
		upper_green = np.array([70,255,255])

		# create a mask for green color using inRange function
		mask = cv2.inRange(hsv, lower_green, upper_green)
		mask = cv2.GaussianBlur(mask, (5, 5), 0)

		# find center of the object
		im2, cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

		if cnts:
			c = max(cnts, key = cv2.contourArea)
			x,y,w,h = cv2.boundingRect(c)

			if w * h > 100: #TODO: tune this parameter!!
				self.goal_seen = True 
				self.pub_goal_pose.publish(self.get_current_pose())

if __name__ == '__main__':
	rospy.init_node('find_turtlebot_and_goal')
	finder = FindTurtlebotAndGoal()
