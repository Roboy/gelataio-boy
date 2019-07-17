#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from dbot_ros_msgs.msg import ObjectState
import tf
from visualization_msgs.msg import InteractiveMarkerUpdate, InteractiveMarkerFeedback, InteractiveMarkerPose

def main():
    # interactive_marker = rospy.Publisher('/interactive_markers/update', InteractiveMarkerUpdate, queue_size=1)
    interactive_marker = rospy.Publisher('/interactive_markers/feedback', InteractiveMarkerFeedback, queue_size=1)

    world_pose = PoseStamped()

    world_pose.pose.position.x = 0
    world_pose.pose.position.y = -1
    world_pose.pose.position.z = 0
    # world_pose.pose.position.x = 0
    # world_pose.pose.position.y = 0
    # world_pose.pose.position.z = 0

    world_pose.pose.orientation.x = 0
    world_pose.pose.orientation.y = 0
    world_pose.pose.orientation.z = 0
    world_pose.pose.orientation.w = 1

    
    # marker_pose = InteractiveMarkerPose()
    # marker_pose.header.frame_id = "world"
    # marker_pose.pose.position.x = world_pose.pose.position.x
    # marker_pose.pose.position.y = world_pose.pose.position.y
    # marker_pose.pose.position.z = world_pose.pose.position.z
    # marker_pose.pose.orientation.x = world_pose.pose.orientation.x
    # marker_pose.pose.orientation.y = world_pose.pose.orientation.y
    # marker_pose.pose.orientation.z = world_pose.pose.orientation.z
    # marker_pose.pose.orientation.w = world_pose.pose.orientation.w
    # marker_pose.name = "arm_right_palm"

    # pos = InteractiveMarkerUpdate()
    pos = InteractiveMarkerFeedback()
    # pos.server_id = "/VRpuppet"
    pos.header.frame_id = 'world'
    pos.marker_name = "arm_left_palm"
    pos.event_type = 5

    # pos.poses.append(marker_pose)


    pos.pose.position.x = world_pose.pose.position.x
    pos.pose.position.y = world_pose.pose.position.y
    pos.pose.position.z = world_pose.pose.position.z
    pos.pose.orientation.x = world_pose.pose.orientation.x
    pos.pose.orientation.y = world_pose.pose.orientation.y
    pos.pose.orientation.z = world_pose.pose.orientation.z
    pos.pose.orientation.w = world_pose.pose.orientation.w
    rospy.loginfo_throttle(1,"%f %f %f"%(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z))
    interactive_marker.publish(pos)



if __name__ == "__main__":

    rospy.init_node('marker_pub', anonymous=True)
    rospy.loginfo("marker pub node running.")

    # 30 FPS = 60 Hz
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        main()
        rate.sleep()