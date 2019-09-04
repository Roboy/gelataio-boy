#!/usr/bin/env python3

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import roboy_middleware_msgs.srv 
import time
import csv
import pickle
from scipy.spatial.transform import Rotation as R

#Getting triad_openvr object with necessary methods to other functions and checks whether vive controller is connected
def _get_triadVR_object(controller_name):
    v = triad_openvr.triad_openvr()
    while controller_name not in v.devices:
        print("Controller not connected or turned off, waiting 5 seconds to reconnect ")
        print(v.devices)
        time.sleep(5)
    return v

#Getting 3d coordinates of the vive controller with respect to the lighthouses
def _get_tracker_coordinates(controller_name):
    v = _get_triadVR_object(controller_name)
    pose_matrix = v.devices[controller_name].get_pose_matrix()
    print(pose_matrix)
    while pose_matrix is None:
        print("Controller is out of lighthouses reach, waiting for controller ")
        time.sleep(1)
        pose_matrix = v.devices[controller_name].get_pose_matrix()
    x = pose_matrix[0][3]
    y = pose_matrix[1][3]
    z = pose_matrix[2][3]
    return np.array([x,y,z])

#Detects the usage of the vertical axis of the trackpad controller
def _get_trackpad_movement(controller_name):
    v = _get_triadVR_object(controller_name)
    return v.devices[controller_name].get_controller_inputs()['trackpad_y']

#Detects the press of the trigger
def _if_trigger_pressed(controller_name):
    v = _get_triadVR_object(controller_name)
    return v.devices[controller_name].get_controller_inputs()['trigger'] > 0.5

#Return the curren angle of a particular robot joint
def _get_single_joint_angle(joint_name, topic_name):
    msg = rospy.wait_for_message(topic_name, JointState)    
    i = msg.name.index(joint_name)
    return msg.position[i]

#Returns the coordinates of a particular robot link
def _get_link_coordinats(link_name, topic_name):
    link_coordinates = np.zeros(3)
    
    def pose_callback(p):
        nonlocal link_coordinates
        if p.header.frame_id == endeffector_name:
            link_coordinates[1] = p.pose.position.y
            link_coordinates[2] = p.pose.position.z
            link_coordinates[0] = p.pose.position.x

    subscriber = rospy.Subscriber(topic_name, PoseStamped, callback=pose_callback)
    
    while np.allclose(link_coordinates, np.zeros(3)):
        rospy.Rate(100).sleep()
    subscriber.unregister()
    return link_coordinates
   
#Creates and returns a JointState object, which includes names, positions. velocities and efforts necessary for every joint
#Adds wrist separately since it's controlled directly
def _get_joint_state(names, angles, wrist_name, wrist_angle):
    joint_state = JointState(Header(),[],[],[],[])
    for i in range(len(names)):
        if names[i] != wrist_name:
            print(wrist_name, names[i])
            joint_state.name.append(names[i])
            joint_state.position.append(angles[i])
            joint_state.velocity.append(0)
            joint_state.effort.append(0)
    joint_state.name.append(wrist_name)
    joint_state.position.append(wrist_angle)
    joint_state.velocity.append(0)
    joint_state.effort.append(0)

    print(joint_state)
    return joint_state

#Returns a default robot JointState object, for a robot with a right arm bent in elbow
def _get_default_joint_state(topic_name, wrist_name):
    msg = rospy.wait_for_message(topic_name, JointState)
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
        
    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    angle_publisher.publish(joint_state)

    initial_position_controller = _get_tracker_coordinates(controller_name)
    initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)

    while not rospy.is_shutdown():
        if _if_trigger_pressed(controller_name):
            joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
            initial_position_controller = _get_tracker_coordinates(controller_name)
            initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

        else:
            current_position_controller = _get_tracker_coordinates(controller_name)
            delta_position_controller = current_position_controller - initial_position_controller
            
            #Rewriting the coordinates from OpenVR's to ROS's coordinate frames
            delta_position_scooper = np.array([
                delta_position_controller[0],
                -delta_position_controller[2],
                delta_position_controller[1]
            ])
            
            modified_position_scooper = initial_position_scooper + delta_position_scooper  
            
            
            wrist_angle = _get_single_joint_angle(wrist_axis_name, joint_state_topic_name) +  _get_trackpad_movement(controller_name)
            
            pos = Point(modified_position_scooper[0], modified_position_scooper[1], modified_position_scooper[2])
            requested_pose = Pose(position = pos)
            ik_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 1, endeffector_name, requested_pose)
            
            try:
                get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)
                ik = get_ik(ik_request)
                joint_state = _get_joint_state(ik.joint_names, ik.angles, wrist_axis_name, wrist_angle)
            except rospy.ServiceException as exc:
                print("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))

        angle_publisher.publish(joint_state)

            
        
        
        

