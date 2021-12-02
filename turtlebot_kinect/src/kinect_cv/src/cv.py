#!/usr/bin/env python

# Import the dependencies as described in example_pub.py
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import imutils
# Define the callback method which is called whenever this node receives a 
# message on its subscribed topic. The received message is passed as the first
# argument to callback().
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

    c = max(cnts, key = cv2.contourArea)
    x,y,w,h = cv2.boundingRect(c)

    # draw the biggest contour (c) in green
    mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB)
    cv2.rectangle(mask,(x,y),(x+w,y+h),(0, 0,255),5)
    cv2.circle(mask, (x + w/2, y + h/2), 4, (0, 255, 0), -1)
    # M = cv2.moments(c)
    # cX = int(M["m10"] / M["m00"])
    # cY = int(M["m01"] / M["m00"])
    # # draw the contour and center of the shape on the image
    # cv2.drawContours(mask, [c], -1, (0, 255, 0), 2)
    # cv2.circle(mask, (cX, cY), 7, (0, 255, 0), -1)
    
    #display the image
    cv2.imshow('image',mask)
    cv2.waitKey(0)

# Define the method which contains the node's main functionality
def listener():

    rospy.Subscriber("/camera/rgb/image_rect_color", Image, callback)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


# Python's syntax for a main() method
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('kinect_processing_node', anonymous=True)

    listener()