#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf2_geometry_msgs
import sys
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped, Pose
from std_msgs.msg import String, Bool, Header
from turtlebot_nav.srv import Hop

#Define the method which contains the main functionality of the node.

class TurtlebotHop:

  def __init__(self):
    # rospy.Subscriber("/turtlebot_next_target", String, self.hop)
    
    #Create a publisher and a tf buffer, which is primed with a tf listener
    self.command_vel_pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)
    self.tfBuffer = tf2_ros.Buffer()
    self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    #Create a service that completes a single hop from one marker to the next
    rospy.Service('turtlebot_hop', Hop, self.hop)

    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    self.r = rospy.Rate(3) # 10hz

    self.Kpx = 0.1
    self.Kpy = 1
    self.Kdx = 0.01
    self.Kdy = -0.3

    self._goal_frame = None

    rospy.spin()

  # def initiate_hop(self, new_frame):
  #     self._goal_frame = new_frame 
  #     self.hop()
  #     self.hop_complete_pub.publish(True)

  def hop(self, request):
    """
    Controls a turtlebot whose position is denoted by turtlebot_frame,
    to go to a position denoted by target_frame
    """

    self._goal_frame = request.next_target

    x_vel = 0
    theta_vel = 0
    startTime = rospy.Time.now()
    last_time = 0
    last_x = 0
    last_y = 0

    self.r.sleep()
    # Loop until the node is killed with Ctrl-C
    goal_time = rospy.Time()
    goal_trans = self.tfBuffer.lookup_transform("base", self._goal_frame, goal_time)

    goal_pose = PoseStamped()
    goal_pose.header = Header()
    goal_pose.header.stamp = goal_time
    goal_pose.header.frame_id = "base" 
    goal_pose.pose = Pose()
    goal_pose.pose.position = goal_trans.transform.translation
    goal_pose.pose.orientation = goal_trans.transform.rotation

    i = 0
    while not rospy.is_shutdown():
      try:
        currTime = rospy.Time.now()
        t = (currTime - startTime).to_sec()
        turtle_trans = self.tfBuffer.lookup_transform("base_link", "base", rospy.Time())
        dt = t - last_time
        last_time = t
        turtle_goal = tf2_geometry_msgs.do_transform_pose(goal_pose, turtle_trans)
        
        # Get state error
        x = turtle_goal.pose.position.x
        y = turtle_goal.pose.position.y


        dist = np.sqrt(x ** 2 + y ** 2)

        log = i % 10 == 0
        if log:
          print("x: %f y: %f dist: %f" % (x, y, dist))

        if dist <= 0.03:
          print("Stopping...")
          self.command_vel_pub.publish(Twist())
          break

        control_command = Twist()

        if np.abs(y) > 0.02:
          theta_vel = self.Kpy * y  #+ self.Kdy * (y - last_y) / dt
          control_command.angular.z = theta_vel
          if log:
            print("Rotating %f" % theta_vel)
        else:
          x_vel = self.Kpx * x # + self.Kdx * (x - last_x) / dt
          control_command.linear.x = x_vel
          if log:
            print("Translating %f" % x_vel)

        # # Generate a control command to send to the robot
        # x_vel = self.Kpx * x # + self.Kdx * (x - last_x) / dt
        # theta_vel = self.Kpy * y  #+ self.Kdy * (y - last_y) / dt
        # control_command = Twist()
        # control_command.linear.x = x_vel
        # control_command.angular.z = theta_vel

        last_x = x
        last_y = y

        self.command_vel_pub.publish(control_command)

        i += 1
      except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
        return False
      # Use our rate object to sleep until it is time to publish again
      self.r.sleep()
    return True

      
# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  rospy.init_node('turtlebot_hop', anonymous=True)

  controller = TurtlebotHop()
  # try:
  #   controller(sys.argv[1])
  # except rospy.ROSInterruptException:
  #   pass