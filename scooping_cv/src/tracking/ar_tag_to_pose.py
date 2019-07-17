#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Pose
from dbot_ros_msgs.msg import ObjectState
import tf
from visualization_msgs.msg import InteractiveMarkerUpdate, InteractiveMarkerFeedback, Marker


class Republisher():
    def __init__(self, *args, **kwargs):
        self.sub = rospy.Subscriber(
            "/visualization_marker", Marker, self.callback)
        self.pub_marker = rospy.Publisher('/interactive_markers/feedback', InteractiveMarkerFeedback, queue_size=1)
        self.marker = InteractiveMarkerFeedback()
        self.tf_listener_ = tf.TransformListener()
        self.pose_prev = Pose()
        return

    def callback(self, data):

        self.marker.header.frame_id = "world"
        self.marker.marker_name = "arm_left_palm"
        self.marker.event_type = 5

        tag_pose = data.pose

        if(abs(tag_pose.position.x - self.pose_prev.position.x) < 0.02 or abs(tag_pose.position.x - self.pose_prev.position.x) > 0.9):
            tag_pose.position.x = self.pose_prev.position.x
        if(abs(tag_pose.position.y - self.pose_prev.position.y) < 0.02 or abs(tag_pose.position.y - self.pose_prev.position.y) > 0.09):
            tag_pose.position.y = self.pose_prev.position.y
        if(abs(tag_pose.position.z - self.pose_prev.position.z) < 0.02 or abs(tag_pose.position.z - self.pose_prev.position.z) > 0.09):
            tag_pose.position.z = self.pose_prev.position.z


        world_pose = PoseStamped()
        world_pose.pose.position.x = -tag_pose.position.x
        world_pose.pose.position.y = -tag_pose.position.z
        world_pose.pose.position.z = -tag_pose.position.y

        world_pose.pose.orientation.x = 0
        world_pose.pose.orientation.y = 0
        world_pose.pose.orientation.z = 0
        world_pose.pose.orientation.w = 1

        self.marker.pose.position.x = world_pose.pose.position.x
        self.marker.pose.position.y = world_pose.pose.position.y
        self.marker.pose.position.z = world_pose.pose.position.z + 0.3
        self.marker.pose.orientation.x = world_pose.pose.orientation.x
        self.marker.pose.orientation.y = world_pose.pose.orientation.y
        self.marker.pose.orientation.z = world_pose.pose.orientation.z
        self.marker.pose.orientation.w = world_pose.pose.orientation.w
        # rospy.loginfo_throttle(1,"%f %f %f"%(self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z))
        rospy.loginfo("%f %f %f"%(self.marker.pose.position.x, self.marker.pose.position.y, self.marker.pose.position.z))



        # pose.header.frame_id = "torso"  # TODO: change this to the camera frame "zed_camera_left"
        # target_frame = 'world'  # or 'world'

        # t = self.tf_listener_.getLatestCommonTime(
        #     target_frame, pose.header.frame_id)

        # # transform stuff
        # world_pose = self.tf_listener_.transformPose(target_frame, pose)
        # print(world_pose)

        # pos = InteractiveMarkerFeedback()
        # pos.header.frame_id = 'world'
        # pos.marker_name = "cup"
        # pos.event_type = 5
        # pos.pose.position.x = world_pose.pose.x
        # pos.pose.position.y = world_pose.pose.y
        # pos.pose.position.z = world_pose.pose.
        # rospy.loginfo_throttle(1,"%f %f %f"%(pos.pose.position.x, pos.pose.position.y, pos.pose.position.z))


        # #create interactive marker
        # self.marker.markers[0].

        # self.pub.publish(pose)
        self.pose_prev = tag_pose
        # self.pub_marker.publish(self.marker)


if __name__ == "__main__":

    rospy.init_node('ar_tag_to_pose', anonymous=True)
    rospy.loginfo("AR tag to pose node running.")

    # 30 FPS = 60 Hz
    rate = rospy.Rate(30)
    repub = Republisher()

    while not rospy.is_shutdown():
        rate.sleep()
