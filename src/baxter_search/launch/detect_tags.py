#!/usr/bin/python

import rospy
from std_msgs.msg import Bool
from ar_track_alvar.ar_track_alvar_msgs import AlvarMarkers

class Detector:

	def __init__(self):
		self.seen_markers = set()
		self.pub = rospy.Publisher('stop_guiding', Bool, queue_size=10)

		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.check_seen)

		rospy.Subscriber("/reset_detector", Bool, self.reset)

	def check_seen(self, msg):
		current = set([m.id for m in msg.markers])
		union = current | self.seen_markers
		if len(union) > len(self.seen_markers):
			self.pub.publish(True)
		self.seen_markers = union 


	def reset(self, msg):
		self.seen_markers = set()
