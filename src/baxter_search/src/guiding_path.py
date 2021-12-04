#!/usr/bin/python

import sys
from baxter_interface import Limb

import rospy
import numpy as np
import traceback

from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import PoseStamped

from path_planner import PathPlanner


def main():
	while not rospy.is_shutdown():
		try:
			planner = PathPlanner("right_arm")
			orien_const = OrientationConstraint()
			orien_const.link_name = "right_gripper"
			orien_const.header.frame_id = "base"
			orien_const.orientation.y = -1.0
			orien_const.absolute_x_axis_tolerance = 0.1
			orien_const.absolute_y_axis_tolerance = 0.1
			orien_const.absolute_z_axis_tolerance = 0.1
			orien_const.weight = 1.0
			# TODO: get this from a topic
			x = 0.47
			y = -0.85
			z = 0.0
			goal = PoseStamped()
			goal.header.frame_id = "base"

			#x, y, and z position
			goal.pose.position.x = x
			goal.pose.position.y = y
			goal.pose.position.z = z

			#Orientation as a quaternion
			goal.pose.orientation.x = 0.0
			goal.pose.orientation.y = -1.0
			goal.pose.orientation.z = 0.0
			goal.pose.orientation.w = 0.0 

			orien_const = OrientationConstraint()
			orien_const.link_name = "right_gripper";
			orien_const.header.frame_id = "base";
			orien_const.orientation.y = -1.0;
			orien_const.absolute_x_axis_tolerance = 0.5;
			orien_const.absolute_y_axis_tolerance = 0.5;
			orien_const.absolute_z_axis_tolerance = 0.5;
			orien_const.weight = 1.0;

			plan = planner.plan_to_pose(goal, [])

			# TODO: delete
			raw_input("Enter to execute plan")

			planner.execute_plan(plan)
		except Exception as e:
			print e
			traceback.print_exc()
		else:
			break

if __name__ == '__main__':
	main()