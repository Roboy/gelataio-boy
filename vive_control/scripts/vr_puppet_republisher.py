#! /usr/bin/env python
    
from roboy_middleware_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

import rospy
import sys
import random

##This script republishes CARDSflow robot arm link poses for VRpuppet

def callback_pose(p):
    requested_id = -1
    if p.header.frame_id == 'arm_right_upper':
        requested_id = 0
    elif p.header.frame_id == 'arm_right_lower': 
        requested_id = 1
    elif p.header.frame_id == 'hand_right_palm':
        requested_id = 2

    #Should be nonnegative in all important cases
    if requested_id > -1:
        constructed_pose = Pose(requested_id, p.pose.position, p.pose.orientation)
        pub.publish(constructed_pose)

def republisher():
    rospy.init_node('robot_state_republisher')
    rospy.Subscriber('robot_state', PoseStamped, callback_pose)
    global pub 
    pub = rospy.Publisher('/roboy_middleware_msgs/Pose', Pose, queue_size=12)
    rospy.spin()

if __name__ == '__main__':
    republisher()
    
    