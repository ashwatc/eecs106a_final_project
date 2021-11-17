#!/usr/bin/env python
import rospy
import sys

if sys.argv[1] == 'sawyer':
	from intera_interface import gripper as robot_gripper
else:
    from baxter_interface import gripper as robot_gripper

rospy.init_node('gripper_test')

# Set up the left gripper
left_gripper = robot_gripper.Gripper('left')

# Calibrate the gripper (other commands won't work unless you do this first)
print('Calibrating...')
left_gripper.calibrate()
rospy.sleep(2.0)

# Close the left gripper
print('Closing...')
left_gripper.close()
rospy.sleep(1.0)

# Open the left gripper
print('Opening...')
left_gripper.open()
rospy.sleep(1.0)
print('Done!')
