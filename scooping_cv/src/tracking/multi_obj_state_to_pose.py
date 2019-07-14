#!/usr/bin/env python

import rospy
import math
import cv2
import numpy as np
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped
from dbot_ros_msgs.msg import ObjectState
import tf
from visualization_msgs.msg import InteractiveMarkerUpdate


class Republisher():
    def __init__(self, *args, **kwargs):
        self.sub = rospy.Subscriber(
            "/multi_object_tracker_service/object_state", ObjectState, self.callback)
        self.pub = rospy.Publisher("/object_pose", PoseStamped, queue_size=1)
        self.pub_marker = rospy.Publisher(
            "/interactive_marker_initializer/update", InteractiveMarkerUpdate, queue_size=1)
        self.marker = InteractiveMarkerUpdate()
        self.tf_listener_ = tf.TransformListener()
        return

    def callback(self, data):

        pose = data.pose
        pose.header.frame_id = "torso"  # TODO: change this to the camera frame "zed_camera_left"
        target_frame = 'world'  # or 'world'

        t = self.tf_listener_.getLatestCommonTime(
            target_frame, pose.header.frame_id)

        # transform stuff
        world_pose = self.tf_listener_.transformPose(target_frame, pose)
        print(world_pose)

        # #create interactive marker
        # self.marker.markers[0].

        self.pub.publish(pose)


if __name__ == "__main__":

    rospy.init_node('multi_obj_state_to_pose', anonymous=True)
    rospy.loginfo("Object state to pose node running.")

    # 30 FPS = 60 Hz
    rate = rospy.Rate(30)
    repub = Republisher()

    while not rospy.is_shutdown():
        rate.sleep()
