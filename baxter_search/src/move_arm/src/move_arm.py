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
    arm = 'right'
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    if robo == 'sawyer':
    	arm = 'right'

    if robo == 'sawyer':
        from intera_interface import gripper as robot_gripper
    else:
        # from baxter_interface import gripper as robot_gripper
        from baxter_interface import camera as robot_camera
    
    # # right_gripper = robot_gripper.Gripper('right')
    # right_camera = robot_camera.CameraController('right')

    print("\n\nwe got here baby\n\n")



    # pose_positions = [[-0.018, -0.933, 0.207, -0.683, 0.730, 0.022, 0.022], [0.295, -0.983, 0.225, -0.679, 0.696, -0.179, 0.148], 
    # [0.412, -0.979, 0.288, 0.678, -0.657, 0.238, -0.227], [0.504, -0.968, 0.335, 0.670, -0.624, 0.288, -0.280], [0.723, -0.901, 0.405, -0.586, 0.635, -0.375, 0.335]]

    pose_positions = [[0.013, -0.826, 0.305, -0.018, 1.000, -0.010, 0.009], [0.197, -0.836, 0.361, 0.004, 0.980, -0.048, 0.193],
    [0.354, -0.797, 0.402, -0.003, 0.959, -0.053, 0.277], [0.568, -0.727, 0.482, -0.011, 0.903, -0.022, 0.430]]


    raw_input('Press [ Enter ]: ')
    # right_gripper.calibrate()
    # right_gripper.open()
    rospy.sleep(1.0)
    
    for i, pose in enumerate(pose_positions):
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = arm + "_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = arm + "_hand_camera"
        if robo == 'sawyer':
        	link += '_tip'

        request.ik_request.ik_link_name = link
        request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = pose[0]
        request.ik_request.pose_stamped.pose.position.y = pose[1]
        request.ik_request.pose_stamped.pose.position.z = pose[2]      

        norm = np.sqrt(sum([p**2 for p in pose[3:]]))
        print("\n\nNORM VAL:", norm, "\n\n")

        request.ik_request.pose_stamped.pose.orientation.x = pose[3] / norm
        request.ik_request.pose_stamped.pose.orientation.y = pose[4] / norm
        request.ik_request.pose_stamped.pose.orientation.z = pose[5] / norm
        request.ik_request.pose_stamped.pose.orientation.w = pose[6] / norm
        # request.ik_request.pose_stamped.pose.orientation.x = 0.0
        # request.ik_request.pose_stamped.pose.orientation.y = 1.0
        # request.ik_request.pose_stamped.pose.orientation.z = 0.0
        # request.ik_request.pose_stamped.pose.orientation.w = 0.0
        
        try:
            # Send the request to the service
            response = compute_ik(request)

            print("\n\n got ik response \n\n")
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander(arm + "_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)
            
            # # TEMP CODE ASHWAT TESTING
            # group.set_planning_time(10);

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            # Plan IK and execute
            group.go()
            
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
    
        #if i == 1:
            #left_gripper.close()
        #if i == 4:
            #left_gripper.open()
        rospy.sleep(1.0)
    
    
# Python's syntax for a main() method
if __name__ == '__main__':
    main(sys.argv[1])