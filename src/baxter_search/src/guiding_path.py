#!/usr/bin/python

import sys
from baxter_interface import Limb

import rospy
import numpy as np
import traceback
import tf2_ros

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from path_planner import PathPlanner

CAMERA_LINK = "right_hand_camera"

class Guide:

	def __init__(self, group="right_arm", end_effector=CAMERA_LINK, speed=0.25):
		self.planner = PathPlanner("right_arm", end_effector, 0.25)

		self.end_ef = end_effector

		orien_const = OrientationConstraint()
		orien_const.link_name = end_effector
		orien_const.header.frame_id = "base"
		orien_const.orientation.x = np.sqrt(2) / 2
		orien_const.orientation.y = -np.sqrt(2) / 2
		orien_const.absolute_x_axis_tolerance = np.pi / 4
		orien_const.absolute_y_axis_tolerance = np.pi / 4
		orien_const.absolute_z_axis_tolerance = np.pi
		orien_const.weight = 1.0;

		self.orien_const = orien_const

		# TODO: get this from a topic
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

		self.goal = goal

		self.tfBuffer = tf2_ros.Buffer()
  		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

  		self._stop = False
  		rospy.Subscriber("/baxter_guiding", Bool, lambda msg: self.set_stop(msg.data))

 	def set_stop(self, start_guiding):
 		self._stop = not start_guiding


  	def run(self):
		while not rospy.is_shutdown():
			try:
				print("\nPlanning path to goal pose")
				plan = self.planner.plan_to_pose(self.goal, [self.orien_const])
				
				if not plan.joint_trajectory.points:
					print("Failed to find trajectory, retrying")
					continue

				raw_input("Enter to execute plan")

				self.planner.execute_plan(plan)
				
				# subscribe to topic telling us when to stop
				close_to_goal = False

				while not close_to_goal and not self._stop:
					curr_trans = self.tfBuffer.lookup_transform("base", self.end_ef, rospy.Time())
					close_to_goal = np.linalg.norm([curr_trans.transform.translation.x - self.goal.pose.position.x, curr_trans.transform.translation.y - self.goal.pose.position.y, curr_trans.transform.translation.z - self.goal.pose.position.z]) <= 0.02
				if self._stop:
					self.planner.stop_execution()
					self.set_stop(False)
					continue
				break

			except Exception as e:
				print e
				traceback.print_exc()


if __name__ == '__main__':
	rospy.init_node('guide_arm')
	guide = Guide()
	guide.run()