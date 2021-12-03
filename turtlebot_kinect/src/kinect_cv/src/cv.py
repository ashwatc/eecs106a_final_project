#!/usr/bin/env python

# Import the dependencies as described in example_pub.py
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
from geometry_msgs.msg import Twist


Kppos = 0.001
Kptheta = -0.001
pos_vel = 0
theta_vel = 0
last_time = 0
start_time = 0
Kdpos = 0.1
Kdtheta = -0.3
last_pos = 0
last_theta = 0

# Define the callback method which is called whenever this node receives a 
# message on its subscribed topic. The received message is passed as the first
# argument to callback().

pub = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

def callback(message):
    # Print the contents of the message to the console
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

    # # draw the biggest contour (c) in green
    # mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    # cv2.rectangle(mask,(x,y),(x+w,y+h),(0, 0,255),5)
    # cv2.circle(mask, (x + w/2, y + h/2), 4, (0, 255, 0), -1)
    # print("x: ", x + w/2)
    # print("y: ", y + h/2)
    # print("area:", w*h)
    # print("\n")
    # # #display the image
    # cv2.imshow('image',mask)
    # cv2.waitKey(0)
    #-----------------------------------------------------------------------------

    #Create a publisher and a tf buffer, which is primed with a tf listener
  
    # Create a timer object that will sleep long enough to result in
    # a 10Hz publishing rate
    r = rospy.Rate(10) # 10hz
    
    global last_time, pub
    global last_pos
    global last_theta


    # goal_frame = 'ar_'
    r.sleep()
    # Loop until the node is killed with Ctrl-C
    try:
        currTime = rospy.Time.now()
        t = (currTime - start_time).to_sec()
        dt = t - last_time
        # print(last_time, t, dt)
        # Get error
        pos = (mask.shape[0] * mask.shape[1]) / (w * h)
        print(w*h, mask.shape)
        theta = ((x + w/2) - (mask.shape[1] / 2))
        # print(theta)

        # Generate a control command to send to the robot
        pos_vel = Kppos * pos #+ Kdpos * (pos - last_pos) / dt
        theta_vel = Kptheta * theta  #+ Kdtheta * (theta - last_theta) / dt
        # print(pos_vel)
        # print(theta_vel)

        print(pos_vel)
        print("\n")
        control_command = Twist()
        
        if w * h < 1000:
            control_command.angular.z = 0.6
            control_command.linear.x = 0.0 #pos_vel

        else:
            control_command.angular.z = theta_vel
            if w * h < 15000:
                control_command.linear.x = pos_vel
            else:
                control_command.linear.x = 0
                control_command.angular.z = 0


        last_pos = pos
        last_theta = theta
        last_time = t
        #################################### end your code ###############

        pub.publish(control_command)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        print(e)
    # Use our rate object to sleep until it is time to publish again
    r.sleep()
# Define the method which contains the node's main functionality
def listener():

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


# Python's syntax for a main() method
if __name__ == '__main__':
    global start_time
    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('kinect_processing_node', anonymous=True)
    start_time = rospy.Time.now()
    listener()