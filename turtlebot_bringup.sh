#!/bin/bash

ssh turtlebot@pink.local < export ROS_MASTER_URI=http://archytas.local:11311; roslaunch turtlebot_bringup minimal.launch

