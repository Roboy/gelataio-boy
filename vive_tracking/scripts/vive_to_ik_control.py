#!/usr/bin/env python

import triad_openvr
import time
import rospy
import numpy as np
import math
import std_msgs
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from roboy_middleware_msgs.msg import MotorCommand
import roboy_middleware_msgs.srv
from geometry_msgs.msg import PoseStamped
import time
import csv
import pickle
from scipy.spatial.transform import Rotation as R
import keyboard

v = triad_openvr.triad_openvr()
take_control = False
link_coordinates = [0,0,0]
palm_pose = [0,0,0]

#Getting 3d coordinates of the vive controller with respect to the lighthouses
def _get_tracker_coordinates(controller_name):
    global v
    pose_matrix = v.devices[controller_name].get_pose_matrix()
    print(pose_matrix)
    if pose_matrix is None:
        return [0,0,0]
    pose_matrix = v.devices[controller_name].get_pose_matrix()
    x = pose_matrix[0][3]
    y = pose_matrix[1][3]
    z = pose_matrix[2][3]
    return np.array([x,y,z])

#Detects the usage of the vertical axis of the trackpad controller
def _get_trackpad_movement(controller_name):
    global v
    return v.devices[controller_name].get_controller_inputs()['trackpad_y']

#Detects the press of the trigger
def _if_trigger_pressed(controller_name):
    global v
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
        global link_coordinates
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

def take_control_srv(req):
    global take_control
    take_control = req.data
    if take_control:
        return True, "taking control"
    else:
        return True, "giving control back"

def pose_cb(data):
    if data.header.frame_id == "palm_right":
        global palm_pose
        palm_pose = [data.pose.position.x, data.pose.position.y, data.pose.position.z]

if __name__ == "__main__":
    controller_name = "controller_1"
    endeffector_name = "scooper"
    inverse_kinematics_service = "ik"
    robot_state_topic = "robot_state"
    target_joint_angle_topic = "joint_targets"
    wrist_axis_name = "wrist_right"
    joint_state_topic_name =  "cardsflow_joint_states"
    global v
    global palm_pose
    rospy.init_node('vive_to_ik_publisher')
    requested_position_topic_name =  "/vive/requested_position"
    vive_joint_state_topic_name = '/vive/ik_joint_state'
    
    angle_publisher = rospy.Publisher('/joint_targets', JointState, queue_size=5)
    s = rospy.Service('take_control', SetBool, take_control_srv)

    joint_state = _get_default_joint_state(joint_state_topic_name, wrist_axis_name)
    
    wrist_publisher = rospy.Publisher('roboy/middleware/MotorCommand', MotorCommand)
    wrist_angle = 0.2

    robot_state_sub = rospy.Subscriber('/robot_state', PoseStamped, pose_cb)
    initial_position_scooper = [0.5,0,0]
    initial_position_controller = [0,0,0]
    modified_position_scooper_prev = [0,0,0]

    t0 = rospy.Time.now()
    wrist_angle_prev = 0.2
    while not rospy.is_shutdown():
        if(take_control):
            if((rospy.Time.now()-t0).to_sec()>0.1):
                t0 = rospy.Time.now()
                wrist_angle = (v.devices[controller_name].get_controller_inputs()['trackpad_y']+1)/2
                wrist_angle = wrist_angle*180
                if(wrist_angle<20):
                    wrist_angle = 20
                if(wrist_angle>160):
                    wrist_angle = 160
                if(wrist_angle_prev is not wrist_angle):
                    rospy.loginfo_throttle(1,wrist_angle)
                    wrist_publisher.publish(MotorCommand(6, [2], [wrist_angle]))
                    wrist_angle_prev = wrist_angle


            if v.devices[controller_name].get_controller_inputs()['trigger']>0.5:
                pose_matrix = v.devices[controller_name].get_pose_matrix()
                if pose_matrix is None:
                    initial_position_scooper = [0.5,0,0]
                    continue
                initial_position_controller = [pose_matrix[0][3],pose_matrix[1][3],pose_matrix[2][3]]
                initial_position_scooper = palm_pose
                rospy.loginfo_throttle(0.1,initial_position_scooper)
                # rospy.loginfo_throttle(0.1,palm_pose)
            else:
                pose_matrix = v.devices[controller_name].get_pose_matrix()
                rospy.loginfo_throttle(1,pose_matrix)
                if pose_matrix is None:
                    continue
                x = pose_matrix[0][3]
                y = pose_matrix[1][3]
                z = pose_matrix[2][3]

                current_position_controller = np.array([x,y,z])
                diff_controller = current_position_controller-initial_position_controller
                modified_position_scooper = [initial_position_scooper[0]+diff_controller[2],initial_position_scooper[1]+diff_controller[0],initial_position_scooper[2]+diff_controller[1]]

                new_pos = [0.6*modified_position_scooper_prev[0]+ 0.4*modified_position_scooper[0],
                                             0.6*modified_position_scooper_prev[1]+ 0.4*modified_position_scooper[1],
                                             0.6*modified_position_scooper_prev[2]+ 0.4*modified_position_scooper[2]]
                modified_position_scooper_prev = modified_position_scooper

                rospy.loginfo_throttle(0.1,new_pos)

                pos = Point(new_pos[0], new_pos[1], new_pos[2])
                requested_pose = Pose(position = pos)
                ik_request = roboy_middleware_msgs.srv.InverseKinematicsRequest(endeffector_name, 1, endeffector_name, requested_pose)

                try:
                    get_ik = rospy.ServiceProxy(inverse_kinematics_service, roboy_middleware_msgs.srv.InverseKinematics)
                    ik = get_ik(ik_request);
                    joint_state = _get_joint_state(ik.joint_names, ik.angles, wrist_axis_name)
                except rospy.ServiceException as exc:
                    print("Inverse Kinematics calculation wasn't successfull\n: " + str(exc))

                angle_publisher.publish(joint_state)
        else:
            rospy.loginfo_throttle(5,"its really not in my hands")
        # if keyboard.is_pressed('s'):  # if key 's' is pressed
        #     take_control = not take_control
        #     if(take_control):
        #         print('taking control')
        #     else:
        #         print('giving control back')
        # else:
        #     print('press s or go to hell')


            
        
        
        

