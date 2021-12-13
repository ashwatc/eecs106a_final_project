#!/usr/bin/python

import rospy

from turtlebot_nav.srv import Rescue
import tf2_ros


from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
from geometry_msgs.msg import Twist

class TurtleFinalCountdown:
	def __init__(self):
		#Create a service that completes a single hop from one marker to the next
		rospy.Service('turtlebot_last_leg', Rescue, self.rescue)
		self.Kppos = 0.003
		self.Kptheta = -0.001
		self.pos_vel = 0
		self.theta_vel = 0
		self.last_time = 0
		self.start_time = rospy.Time.now()
		self.Kdpos = 0.1
		self.Kdtheta = -0.3
		self.last_pos = 0
		self.last_theta = 0

		self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

		self.still_rescuing = True

		# subscribe to the camera and keep storing velocities

		# Create a timer object that will sleep long enough to result in
		# a 10Hz publishing rate
		r = rospy.Rate(10) # 10hz

		print("initialized service")
		# print(Rescue)

		rospy.spin()

	def callback(self, message):
		#do cv stuff

		bridge = CvBridge()
		cv_img = bridge.imgmsg_to_cv2(message, desired_encoding='passthrough')

		hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)


		#set the lower and upper bounds for the green hue
		lower_green = np.array([50,100,50])
		upper_green = np.array([70,255,255])

		#create a mask for green color using inRange function
		mask = cv2.inRange(hsv, lower_green, upper_green)
		mask = cv2.GaussianBlur(mask, (5, 5), 0)

		#find center of the object
		im2, cnts, hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)

		if not cnts:
		    return

		c = max(cnts, key = cv2.contourArea)
		x,y,w,h = cv2.boundingRect(c)

		# Create a timer object that will sleep long enough to result in
		# a 10Hz publishing rate
		r = rospy.Rate(10) # 10hz

		r.sleep()
		# Loop until the node is killed with Ctrl-C
		try:
			currTime = rospy.Time.now()
			t = (currTime - self.start_time).to_sec()
			dt = t - self.last_time
			# Get error
			pos = (mask.shape[0] * mask.shape[1]) / (w * h)
			print(w*h, mask.shape)
			theta = ((x + w/2) - (mask.shape[1] / 2))

			# Generate a control command to send to the robot
			pos_vel = self.Kppos * pos #+ self.Kdpos * (pos - self.last_pos) / dt
			theta_vel = self.Kptheta * theta  #+ self.Kdtheta * (theta - self.last_theta) / dt

			print(pos_vel)
			print("\n")
			control_command = Twist()
			print(w * h)
	        
			if w * h < 5000:
				control_command.angular.z = 0.6
				control_command.linear.x = 0.0 #pos_vel

			else:
				control_command.angular.z = theta_vel
				if w * h < 45000:
					control_command.linear.x = pos_vel
				else:
					control_command.linear.x = 0
					control_command.angular.z = 0
					self.still_rescuing = False


			self.last_pos = pos
			self.last_theta = theta
			self.last_time = t

			self.pub.publish(control_command)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
			print(e)
	    # Use our rate object to sleep until it is time to publish again
		r.sleep()


	def rescue(self, request):
		print("HERE")
		self.start_time = rospy.Time.now()
		self.image_listening = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback)
		self.still_rescuing = True
		while self.still_rescuing:
			continue
		self.image_listening.unregister()
		return True

if __name__ == '__main__':
 	rospy.init_node('turtlebot_last_leg', anonymous=True)

	controller = TurtleFinalCountdown()
