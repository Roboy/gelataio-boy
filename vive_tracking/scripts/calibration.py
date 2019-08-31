#!/usr/bin/env python3

import triad_openvr
import rospy
import numpy as np
import math
from sklearn import linear_model
from pyquaternion import Quaternion
import csv

rospy.init_node('tracker_tf_broadcaster')

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


# Calculates rotation matrix to euler angles
def rotationMatrixToEulerAngles(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        z = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        x = math.atan2(R[1,0], R[0,0])
    else :
        z = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        x = 0

    return np.array([x, y, z])



def get_quarterion(device_name, pose):
    try:
        pose = v.devices[device_name].get_pose_quaternion()
    except:
        rospy.loginfo("could not find transform world->"+ device_name +", initialization might be wrong")

    q_tracker = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init1.inverse
    return q_tracker

def _get_euler_angles_difference(tracker_1_name, tracker_2_name, tracker_1_pose, tracker_2_pose):
    q_tracker_1 = get_quarterion(tracker_1_name, tracker_1_pose)
    q_tracker_2 = get_quarterion(tracker_2_name, tracker_2_pose)

    q_tracker_diff = q_tracker_2*q_tracker_1.inverse

    euler_angles = rotationMatrixToEulerAngles(q_tracker_diff.rotation_matrix)

    return euler_angles


if __name__ == "__main__":
    track_elbow = False
    track_writst = False

    torso_tracker_name = "tracker_1"
    shoulder_tracker_name = "tracker_2"
    forearm_tracker_name = "tracker_3"
    palm_tracker_name = "tracker_5"

    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()


    #Joint values for cardsflow
    relaxed_values = [0, 0, 0]
    side_values = [0.0, 1.57, 0.0]
    fist_up_values = [0.0, 1.57, -1.57]
    arm_back_values = [1.57, 0.79, 0]
    front_values = [-1.57, 0, -1.57]
    arm_up_values = [0.0, 3.14, 1.57]

    elbow_relaxed = 0.0
    elbow_bent = 1.57

    palm_down = 0.0
    palm_right = 1.57
    palm_left = 1.57



    input("Relax your arm, put it down parallel to your torso with the palm facing towards you")

    initial_pose_torso = v.devices[torso_tracker_name].get_pose_quaternion()
    initial_pose_shoulder = v.devices[shoulder_tracker_name].get_pose_quaternion()
    if track_elbow is True:
        initial_pose_forearm =  v.devices[forearm_tracker_name].get_pose_quaternion()
    if track_writst is True:
        initial_pose_palm =  v.devices[palm_tracker_name].get_pose_quaternion()

    input("Point your right arm to the right, orthogonal to your body and bend your elbow so your hand points in front of you")

    euler_side_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)
    
    if track_writst is True:
        input("Point your palm down")
        euler_down_palm = _get_euler_angles_difference(forearm_tracker_name, palm_tracker_name, initial_pose_forearm, initial_pose_palm)
        input("Point your palm left")
        euler_left_palm = _get_euler_angles_difference(forearm_tracker_name, palm_tracker_name, initial_pose_forearm, initial_pose_palm)
        input("Point your palm right ")
        euler_right_palm = _get_euler_angles_difference(forearm_tracker_name, palm_tracker_name, initial_pose_forearm, initial_pose_palm)

    if track_elbow is True:
        euler_elbow_bent = _get_euler_angles_difference(torso_tracker_name, forearm_tracker_name, initial_pose_shoulder, initial_pose_forearm)
        input("Straighten your elbow ")
        euler_elbow_straight = _get_euler_angles_difference(torso_tracker_name, forearm_tracker_name, initial_pose_shoulder, initial_pose_forearm)

    input("Point your right arm back at 45 degrees, and bend your elbow so the whole surface of your arm is parallel to the ground")

    euler_arm_back_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm to the right, orthogonal to your body and bend your elbow so your hand points up")

    euler_fist_up_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm upwards, parallel to your body and bend your elbow so your hand points left")

    euler_arm_up_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm infront of you, orthogonal to your body and parallel to the ground, and bend your elbow so your hand points left")

    euler_front_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)
    
    input("Relax your arm, put it down parallel to your torso with the palm facing towards you")

    euler_relaxed_shoulder = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    reg_shoulder = linear_model.LinearRegression()
    reg_shoulder.fit([euler_relaxed_shoulder, euler_side_shoulder, euler_arm_back_shoulder, euler_fist_up_shoulder, euler_front_shoulder, euler_arm_up_shoulder],
                    [relaxed_values, side_values, arm_back_values, fist_up_values, front_values, arm_up_values])

    csvfile =  open('calibration.csv', 'w')
    writer = csv.writer(csvfile)

    #[array([0.6578727 , 0.45521055, 1.12019768]), array([ 0.24424409, -0.02194311,  0.07315386]), array([ 0.31908318,  0.43454081, -0.00398665]), array([ 0.94450428, -0.07541627,  0.07306826]), array([-0.43969506, -0.24915067,  0.81210489]), array([ 0.26728001, -1.29651049,  0.26622031])]
    print ([euler_relaxed_shoulder, euler_side_shoulder, euler_arm_back_shoulder, euler_fist_up_shoulder, euler_front_shoulder, euler_arm_up_shoulder])
    writer.writerow (np.concatenate((reg_shoulder.coef_[0], np.array([0,0,0,0,0,0]), np.array([reg_shoulder.intercept_[0]]))))
    writer.writerow (np.concatenate((reg_shoulder.coef_[1], np.array([0,0,0,0,0,0]), np.array([reg_shoulder.intercept_[1]]))))
    writer.writerow (np.concatenate((reg_shoulder.coef_[2], np.array([0,0,0,0,0,0]), np.array([reg_shoulder.intercept_[2]]))))
    
    if track_elbow is True:
        reg_elbow = linear_model.LinearRegression()
        reg_elbow.fit([euler_elbow_straight, euler_elbow_bent], [elbow_relaxed, elbow_bent])
        writer.writerow (np.concatenate((np.array([0,0,0]), reg_elbow.coef_, np.array([0,0,0]), np.array([reg_elbow.intercept_]))))
    else:
        writer.writerow (np.array([0,0,0,0,0,0,0,0,0,0]))

    if track_writst is True:
        reg_writst = linear_model.LinearRegression()
        reg_writst.fit([euler_down_palm, euler_left_palm, euler_right_palm], [palm_down, palm_left, palm_right])
        writer.writerow (np.concatenate((np.array([0,0,0,0,0,0]), reg_writst.coef_, np.array([reg_writst.intercept_]))))
    else:
        writer.writerow (np.array([0,0,0,0,0,0,0,0,0,0]))
