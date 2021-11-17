#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys

def main(robo):
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    arm = 'left'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'

    if robo == 'sawyer':
        from intera_interface import gripper as robot_gripper
    else:
        from baxter_interface import gripper as robot_gripper
    
    left_gripper = robot_gripper.Gripper('left')
    
    pose_positions = [[0.774, 0.223, 0], [0.774, 0.223, -0.213], [0.774, 0.223, 0], [0.720, -0.081, 0], [0.720, -0.081, -0.203], [0.720, -0.081, 0], [0.625, 0.144, 0.312]]
    raw_input('Press [ Enter ]: ')
    left_gripper.calibrate()
    left_gripper.open()
    rospy.sleep(1.0)
    
    for i, pose in enumerate(pose_positions):
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_gripper"
        if robo == 'sawyer':
        	link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = pose[0]
        request.ik_request.pose_stamped.pose.position.y = pose[1]
        request.ik_request.pose_stamped.pose.position.z = pose[2]        
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
        if i == 1:
            left_gripper.close()
        if i == 4:
            left_gripper.open()
        # rospy.sleep(1.0)
    
    
# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])

