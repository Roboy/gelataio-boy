#!/usr/bin/env python3

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
import yaml
import os.path
import sys
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
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
def _get_tracker_coordinates(controller_name, triad_openvr_object):
    pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    print(pose_matrix)
    while pose_matrix is None:
        print("Controller is out of lighthouses reach, waiting for controller ")
        time.sleep(1)
        pose_matrix = triad_openvr_object.devices[controller_name].get_pose_matrix()
    x = pose_matrix[0][3]
    y = pose_matrix[1][3]
    z = pose_matrix[2][3]
    return np.array([x,y,z])

#Detects the usage of the vertical axis of the trackpad controller
def _get_trackpad_movement(controller_name, triad_openvr_object):
    return triad_openvr_object.devices[controller_name].get_controller_inputs()['trackpad_y']

#Detects the press of the trigger
def _if_trigger_pressed(controller_name, triad_openvr_object):
    return triad_openvr_object.devices[controller_name].get_controller_inputs()['trigger'] > 0.5

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
def _get_joint_state(names, angles, wrist_name):
    joint_state = JointState(Header(),[],[],[],[])
    for i in range(len(names)):
        if names[i] != wrist_name:
            print(wrist_name, names[i])
            joint_state.name.append(names[i])
            joint_state.position.append(angles[i])
            joint_state.velocity.append(0)
            joint_state.effort.append(0)
    print(joint_state)
    return joint_state

#Returns a default robot JointState object, for a robot with a right arm bent in elbow
def _get_default_joint_state(topic_name, wrist_name):
    msg = rospy.wait_for_message(topic_name, JointState)
    angles = [1.57 if i == "elbow_right" else 0 for i in msg.name]

    return _get_joint_state(msg.name, angles, wrist_name)

def joint_state_filtering(alpha, joint_state, prev_joint_state, wrist_name):
    name_list = []
    angle_list = []
    for i in range(len(joint_state.name)):
        if joint_state.name[i] in prev_joint_state.name:
            i_prev = prev_joint_state.name.index(joint_state.name[i])
            name_list.append(joint_state.name[i])
            angle_list.append(  alpha * prev_joint_state.position[i_prev] + 
                                (1-alpha) * joint_state.position[i])
        else:
            name_list.append(joint_state.name[i])
            angle_list.append(joint_state.position[i])
    return _get_joint_state(name_list, angle_list, wrist_name)


if __name__ == "__main__":

    # Assignment of config variables from yaml config for next with default cases for everything for the next ~70 lines
    # Just scroll through it, it's boring
    # Or look at their detailed description in config directory

    if len(sys.argv) > 1:
        config_name = sys.argv[1]
    else:
        config_name = "vive_control.yaml"

    current_path = os.path.abspath(os.path.dirname(__file__))
    config_path = os.path.join(current_path, "../config/", config_name)

    with open(config_path) as config_file:
        config_dict = yaml.load(config_file, Loader=yaml.FullLoader)        
    
 

    if "controller_name" in config_dict:
        controller_name = config_dict[controller_name]
    else:
        controller_name = "controller_1"

    if "endeffector_name" in config_dict:
        endeffector_name = config_dict[endeffector_name]
    else:
        endeffector_name = "scooper"

    if "inverse_kinematics_service" in config_dict:
        inverse_kinematics_service = config_dict[inverse_kinematics_service]
    else:
        inverse_kinematics_service = "ik"

    if "robot_state_topic" in config_dict:
        robot_state_topic = config_dict[robot_state_topic]
    else:
        robot_state_topic = "robot_state"

    if "target_joint_angle_topic" in config_dict:
        target_joint_angle_topic = config_dict[target_joint_angle_topic]
    else:
        target_joint_angle_topic = "joint_targets"

    if "wrist_axis_name" in config_dict:
        wrist_axis_name = config_dict[wrist_axis_name]
    else:
        wrist_axis_name = "wrist_right"

    if "joint_state_topic_name" in config_dict:
        joint_state_topic_name = config_dict[joint_state_topic_name]
    else:
        joint_state_topic_name = "cardsflow_joint_states"

    if "wrist_motor_topic" in config_dict:
        wrist_motor_topic = config_dict[wrist_motor_topic]
    else:
        wrist_motor_topic = "roboy/middleware/MotorCommand"

    if "wrist_simulated" in config_dict:
        wrist_simulated = config_dict[wrist_simulated]
    else:
        wrist_simulated = True

    if "wrist_direct_motor_control" in config_dict:
        wrist_direct_motor_control = config_dict[wrist_direct_motor_control]
    else:
        wrist_direct_motor_control = True

    if "filtering_alpha" in config_dict:
        filtering_alpha = config_dict[filtering_alpha]
    else:
        filtering_alpha = 0.4
    
    if "wrist_min_angle" in config_dict:
        wrist_min_angle = config_dict[wrist_min_angle]
    else:
        wrist_min_angle = 20
        
    if "wrist_max_angle" in config_dict:
        wrist_max_angle = config_dict[wrist_max_angle]
    else:
        wrist_max_angle = 160

    if "sensitivity" in config_dict:
        sensitivity = config_dict[sensitivity]
    else:
        sensitivity = 1

    rospy.init_node('vive_to_ik_publisher')

    wrist_publisher = rospy.Publisher(wrist_motor_topic, MotorCommand)
    angle_publisher = rospy.Publisher(target_joint_angle_topic, JointState, queue_size=5)

    v = _get_triadVR_object(controller_name)

    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    angle_publisher.publish(joint_state)

    initial_position_controller = _get_tracker_coordinates(controller_name, v)
    initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    prev_joint_state = joint_state

    while not rospy.is_shutdown():
        if _if_trigger_pressed(controller_name, v):
            joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
            prev_joint_state = joint_state
            initial_position_controller = _get_tracker_coordinates(controller_name, v)
            initial_position_scooper = _get_link_coordinats(endeffector_name, robot_state_topic)

        else:
            current_position_controller = _get_tracker_coordinates(controller_name, v)
            delta_position_controller = current_position_controller - initial_position_controller
            
            #Rewriting the coordinates from OpenVR's to ROS's coordinate frames
            delta_position_scooper = sensitivity * np.array([
                delta_position_controller[0],
                -delta_position_controller[2],
                delta_position_controller[1]
            ])
            


            modified_position_scooper = initial_position_scooper + delta_position_scooper  
            
            
            pos = Point(modified_position_scooper[0], modified_position_scooper[1], modified_position_scooper[2])
            requested_pose = Pose(position = pos)
            ik_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 1, endeffector_name, requested_pose)
            
            try:
                get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)
                ik = get_ik(ik_request)
                new_joint_state = _get_joint_state(ik.joint_names, ik.angles, wrist_axis_name)
                joint_state = joint_state_filtering(filtering_alpha, joint_state, prev_joint_state, wrist_axis_name)
                prev_joint_state = new_joint_state
            except rospy.ServiceException as exc:
                print("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))

        wrist_angle = _get_single_joint_angle(wrist_axis_name, joint_state_topic_name) +  _get_trackpad_movement(controller_name, v)
        if(wrist_angle<wrist_min_angle):
            wrist_angle = wrist_min_angle
        if(wrist_angle>wrist_max_angle):
            wrist_angle = wrist_max_angle
        
        if wrist_simulated:
            joint_state.name.append(wrist_axis_name)
            joint_state.position.append(wrist_angle)
            joint_state.velocity.append(0)
            joint_state.effort.append(0)
        if wrist_direct_motor_control:
            wrist_publisher.publish(MotorCommand(6, [2], [wrist_angle]))

        angle_publisher.publish(joint_state)
