#!/usr/bin/env python3

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import roboy_middleware_msgs.srv 
import time
import csv
import pickle
from scipy.spatial.transform import Rotation as R

def _get_triadVR_object(controller_name):
    v = triad_openvr.triad_openvr()
    while controller_name not in v.devices:
        print("Controller not connected or turned off, waiting 5 seconds to reconnect ")
        time.sleep(5)
    return v

def _get_tracker_coordinates(controller_name):
    v = _get_triadVR_object(controller_name)
    pose_matrix = v.devices[controller_name].get_pose_matrix()
    while pose_matrix is None:
        print("Controller is out of lighthouses reach, waiting for controller ")
        time.sleep(1)
        pose_matrix = v.devices[controller_name].get_pose_matrix()
    x = pose_matrix[0][3]
    y = pose_matrix[1][3]
    z = pose_matrix[2][3]
    return np.array([x,y,z])

def _get_trackpad_movement(controller_name):
    v = _get_triadVR_object(controller_name)
    return v.devices[controller_name].get_controller_inputs()['trackpad_y']

def _if_trigger_pressed(controller_name):
    v = _get_triadVR_object(controller_name)
    return v.devices[controller_name].get_controller_inputs()['trigger'] > 0.5

def _get_single_joint_angle(joint_name, topic_name):
    msg = rospy.wait_for_messages(topic_name, JointState)    
    i = msg.name.index(joint_name)
    return msg.position[i]

def _get_endeffector_coordinats(endeffector_name, topic_name):
    endeffector_cordinates = np.zeros(3)
    
    def pose_callback(p):
        nonlocal endeffector_coordinates
        if p.header.frame_id == endeffector_name:
            endeffector_coordinates[0] = p.pose.position.x
            endeffector_coordinates[1] = p.pose.position.y
            endeffector_coordinates[2] = p.pose.position.z

    subscriber = rospy.Subscriber(topic_name, PoseStamped, callback=pose_callback)
    
    while np.allclose(endeffector_coordinates, np.zeros(3)):
        rospy.Rate(100).sleep()
    subscriber.unregister()
    return endeffector_coordinates
   

def _get_joint_state(names, angles, wrist_name, wrist_angle):
    joint_state = JointState([],[],[],[])
    for i in range(len(names)):
        if names[i] is not wrist_name:
            joint_state.name.append(names[i])
            joint_state.position.append(angles[i])
            joint_state.velocity.append(0)
            joint_state.effort.append(0)
    joint_state.name.append(wrist_name)
    joint_state.position.append(wrist_angle)
    joint_state.velocity.append(0)
    joint_state.effort.append(0)

    return joint_state

def _get_default_joint_state(joint_state_topic, wrist_name):
    msg = rospy.wait_for_messages(topic_name, JointState)
    angles = [1.57 if i == "elbow_right" else 0 for i in msg.name]

    return _get_joint_state(msg.name, angles, wrist_name, 0)

if __name__ == "__main__":
    controller_name = "controller_1"
    endeffector_name = "scooper"
    inverse_kinematics_service = "ik"
    robot_state_topic = "robot_state"
    target_joint_angle_topic = "joint_targets"
    wrist_axis_name = "wrist_right"
    joint_state_topic_name =  "cardsflow_joint_states"

    rospy.init_node('vive_to_ik_publisher')
    angle_publisher = rospy.Publisher('/joint_targets', JointState, queue_size=5)
        

    initial_position_controller = _get_tracker_coordinates(controller_name)
    initial_position_scooper = _get_endeffector_coordinats(endeffector_name, robot_state_topic)

    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)

    while not rospy.is_shutdown():
        if _if_trigger_pressed(controller_name):
            joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
        else:
            current_position_controller = _get_tracker_coordinates(controller_name)
            delta_position_controller = current_position_controller - initial_position_controller
            
            delta_position_scooper = np.array([
                delta_position_controller[0],
                -delta_position_controller[1],
                delta_position_controller[2]
            ])
            
            modified_position_scooper = initial_position_scooper + delta_position_scooper  
            get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)
            wrist_angle = _get_single_joint_angle(wrist_axis_name, joint_state_topic_name) _get_trackpad_movement(controller_name)
            
            pos = Point(modified_position_scooper[0], modified_position_scooper[1], modified_position_scooper[2])
            requested_pose = Pose(position = pos)
            ik_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 0, endeffector_name, requested_pose)
                
            try:
                ik = get_ik(ik_request)
                joint_state = _get_joint_state(ik.joint_names, il.angles, wrist_axis_name, wrist_angle)
            except rospy.ServiceException as exc:
                print("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))

        angle_publisher.publish(joint_state)

            
        
        
        

