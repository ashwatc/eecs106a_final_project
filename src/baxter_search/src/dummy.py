#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

if __name__ == '__main__':
	rospy.init_node('dummy')
	pub = rospy.Publisher('reset_detector', Bool, queue_size=10)
	while not rospy.is_shutdown():
		r = raw_input("reset: ")
		pub.publish(True)
		# if r == 'r':
		# 	pub.publish(True)
		# else:
		# 	pub.publish(False)
