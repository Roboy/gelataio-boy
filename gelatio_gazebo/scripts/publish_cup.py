#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def publish_pose(msg):
    publisher = rospy.Publisher("cup_pose", Pose, queue_size=1)
    i = msg.name.index("icecream_cup")
    pose = msg.pose[i]
    publisher.publish(pose)

if __name__ == '__main__':
    rospy.init_node("cup_pose_translator")
    subscriber = rospy.Subscriber("/gazebo/model_states", ModelStates, publish_pose)
    rospy.spin()
