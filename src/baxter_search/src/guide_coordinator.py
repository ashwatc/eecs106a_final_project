#!/usr/bin/python

import rospy
from std_msgs.msg import Bool, String
from ar_track_alvar_msgs.msg import AlvarMarkers
from turtlebot_nav.srv import Hop

class GuideCoordinator:

	def __init__(self):
		self.seen_markers = set()
		self.baxter_guiding_pub = rospy.Publisher('/baxter_guiding', Bool, queue_size=10)

		rospy.wait_for_service('turtlebot_hop')
		self.turtle_hop_srv = rospy.ServiceProxy('turtlebot_hop', Hop)

		rospy.Subscriber("/begin_guide", Bool, self.begin_guide)

		rospy.spin()

	def check_seen(self, msg):
		current = set([m.id for m in msg.markers])
		union = current | self.seen_markers
		new_markers = current - self.seen_markers
		if len(new_markers) > 0:
			print("Detected new AR tag")
			print(union)

			self.set_baxter_guiding(False)
			self.turtle_hop_srv("ar_marker_%s" % str(new_markers.pop()))
			self.set_baxter_guiding(True)

		self.seen_markers = union 


	def set_baxter_guiding(self, start_guiding):
		self.baxter_guiding_pub.publish(start_guiding)


	def begin_guide(self, msg):
		self.seen_markers = set([5, 0])
		self.set_baxter_guiding(True)
		print("HERE!")
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.check_seen)

if __name__ == '__main__':
	rospy.init_node('guide_coordinator')
	GuideCoordinator()