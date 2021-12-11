#!/usr/bin/python

import rospy
from std_msgs.msg import Bool, String
from ar_track_alvar_msgs.msg import AlvarMarkers
from turtlebot_nav.srv import Hop
from baxter_search.srv import Guide
from geometry_msgs.msg import PoseStamped

class GuideCoordinator:

	def __init__(self):
		self.seen_markers = set()
		self.baxter_guiding_pub = rospy.Publisher('/baxter_guiding', Bool, queue_size=10)

		rospy.wait_for_service('/turtlebot_hop')
		self.turtle_hop_srv = rospy.ServiceProxy('/turtlebot_hop', Hop)

		# rospy.Subscriber("/begin_guide", Bool, self.begin_guide)
		rospy.Service('/guide', Guide, self.start_guide)

		self.pub_goal =rospy.Publisher("/guide_goal", PoseStamped, queue_size=10)

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
			print("Continuing guide\n")
			self.set_baxter_guiding(True)

		self.seen_markers = union 


	def set_baxter_guiding(self, move_baxter):
		self.baxter_guiding_pub.publish(move_baxter)


	def start_guide(self, request):
		self.seen_markers = set(request.ignore_tags)
		self.set_baxter_guiding(True)
		print("HERE!")
		self.pub_goal.publish(request.goal_pose)
		sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.check_seen)
		rospy.wait_for_message("/finished_guide", Bool)
		sub.unregister()
		return True


if __name__ == '__main__':
	rospy.init_node('guide_coordinator')
	GuideCoordinator()