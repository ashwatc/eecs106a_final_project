#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import sys
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

#Define the method which contains the main functionality of the node.

class TurtlebotController:

  def __init__(self):
    rospy.Subscriber("/turtlebot_next_target", Bool, self.initiate_hop)
    
    #Create a publisher and a tf buffer, which is primed with a tf listener
    self.command_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    self.hop_complete_pub = rospy.Publisher('/turtle_hop_complete', Bool, queue_size=10)
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    self.r = rospy.Rate(3) # 10hz

    self.Kpx = 0.1
    self.Kpy = -1
    self.Kdx = 0.1
    self.Kdy = -0.3

    self._goal_frame = None

  def initiate_hop(self, new_frame):
      self._goal_frame = new_frame 
      self.hop()
      self.hop_complete_pub.publish(True)

  def hop(self):
    """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    """

    x_vel = 0
    theta_vel = 0
    startTime = rospy.Time.now()
    last_time = 0
    last_x = 0
    last_y = 0

    r.sleep()
    # Loop until the node is killed with Ctrl-C

    if self.goal_frame:
      while not rospy.is_shutdown():
        try:
          currTime = rospy.Time.now()
          t = (currTime - startTime).to_sec()
          trans = self.tfBuffer.lookup_transform(turtlebot_frame, self.goal_frame, rospy.Time())
          dt = t - last_time
          # print(last_time, t, dt)
          last_time = t
          # Process trans to get your state error
          x = trans.transform.translation.x
          y = trans.transform.translation.y

          # Generate a control command to send to the robot
          x_vel = self.Kpx * x + self.Kdx * (x - last_x) / dt
          theta_vel = self.Kpy * y  #+ self.Kdy * (y - last_y) / dt
          control_command = Twist()
          control_command.linear.x = x_vel
          control_command.angular.z = theta_vel

          last_x = x
          last_y = y
          #################################### end your code ###############

          pub.publish(control_command)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
          print(e)
          break
        # Use our rate object to sleep until it is time to publish again
        r.sleep()

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_controller', anonymous=True)

  controller = TurtlebotController()
  # try:
  #   controller(sys.argv[1])
  # except rospy.ROSInterruptException:
  #   pass