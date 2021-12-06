#!/usr/bin/python

import rospy
from std_msgs.msg import Bool, String
from ar_track_alvar_msgs.msg import AlvarMarkers

class GuideScheduler:

	def __init__(self):
		self.seen_markers = set()
		self.baxter_guiding_pub = rospy.Publisher('/baxter_guiding', Bool, queue_size=10)
		self.turtlebot_target_pub = rospy.Publisher('/turtle_next_target', String, queue_size = 10)

		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.check_seen)

		rospy.Subscriber("/turtle_hop_complete", Bool, self.set_baxter_guiding)

		rospy.Subscriber("/reset_detector", Bool, self.reset_seen)

		rospy.spin()

	def check_seen(self, msg):
		current = set([m.id for m in msg.markers])
		union = current | self.seen_markers
		new_markers = current - self.seen_markers
		if len(new_markers) > 0:
			self.set_baxter_guiding(False)
			self.turtlebot_target_pub.publish("ar_marker_%s" % str(new_markers.pop()))
			print("Detected new AR tag")
			print(union)
			print(new_markers)
		self.seen_markers = union 


	def set_baxter_guiding(self, start_guiding):
		self.baxter_guiding_pub.publish(start_guiding)


	def reset_seen(self, msg):
		self.seen_markers = set()

if __name__ == '__main__':
	rospy.init_node('detect_tags')
	scheduler = GuideScheduler()