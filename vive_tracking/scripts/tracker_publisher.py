#!/usr/bin/env python3

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
from sensor_msgs.msg import JointState
import roboy_middleware_msgs.srv 
import warnings
import csv
import pickle
from scipy.spatial.transform import Rotation as R

def _get_tracker_coordinates(tracker_name):
    v = triad_openvr.triad_openvr()
    pose_matrix = v.devices[controller_name].get_pose_matrix()
    x = pose_mat[0][3]
    y = pose_mat[1][3]
    z = pose_mat[2][3]
    return np.array([x,y,z])

def _get_endeffector_coordinats(endeffector_name, topic_name):
    endeffector_coordinates = np.zeros(3)
    while endeffector_coordinates == np.zeros():
        p = rospy.wait_for_message(topic_name, PoseStamped)
        if p.header.frame_id == endeffector_name:
            endeffector_coordinates[0] = p.pose.position.x
            endeffector_coordinates[1] = p.pose.position.x
            endeffector_coordinates[2] = p.pose.position.x
    return endeffector_coordinates

if __name__ == "__main__":
    controller_name = "controller_1"
    endeffector_name = "scooper"
    inverse_kinematics_service = "\ik"
    robot_state_topic = "robot_state"
    target_joint_angle_topic = "joint_targets"
    

    rospy.init_node('vive_to_ik_publisher')
    angle_publisher = rospy.Publisher('/joint_targets', JointState, queue_size=5)
        

    initial_position_controller = _get_tracker_coordinates(controller_name)
    initial_position_scooper = _get_endeffector_coordinats(endeffector_name, robot_state_topic)

    while not rospy.is_shutdown():
        start = time.time()

        current_position_controller = _get_tracker_coordinates(controller_name)
        delta_position_controller = current_position_controller - initial_position_controller
        
        modified_position_scooper = initial_position_scooper + delta_position_controller        

        rospy.wait_for_service(inverse_kinematics_service)
        get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)

        try:

            requested_pose = Pose(position = modified_position_scooper)
            it_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 0, endeffector_name, requested_pose)
            ik = get_ik(it_request)

            joint_state = JointState([],[],[],[])
            for i in range(len(ik.joint_names)):
                joint_state.name.append(ik.joint_names[i])
                joint_state.position.append(ik.angles[i])
                joint_state.velocity.append(0)
                joint_state.effort.append(0)
            angle_publisher.publish(joint_state)
        
        except rospy.ServiceException as exc:
            warnings.warn("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))
        

