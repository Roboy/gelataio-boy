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
    # use these to change publishing behaviour
    head = False
    shoulder_left = True

    torso_tracker_name = "tracker_2"
    shoulder_tracker_name = "tracker_3"

    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()


    #values for cardsflow
    relaxed_values = [0, 0, 0]
    side_values = [0.0, -1.57, 0.0]
    front_values = [-1.57, 0, -1.57]
    fist_up_values = [0.0, -1.57, 1.57]
    arm_up_values = [0.0, -3.14, 1.57]

    input("Relax your arm, put it down parallel to your torso with the palm facing towards you")

    initial_pose_torso = v.devices["tracker_2"].get_pose_quaternion()
    initial_pose_shoulder = v.devices["tracker_3"].get_pose_quaternion()
    #initial_pose3 = v.devices["tracker_3"].get_pose_quaternion()

    input("Point your right arm to the right, orthogonal to your body and bend your elbow so your hand points in front of you")

    euler_side_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm to the right, orthogonal to your body and bend your elbow so your hand points up")

    euler_fist_up_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm upwards, parallel to your body and bend your elbow so your hand points left")

    euler_arm_up_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    input("Point your right arm infront of you, orthogonal to your body and parallel to the ground, and bend your elbow so your hand points left")

    euler_front_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)
    
    input("Relax your arm, put it down parallel to your torso with the palm facing towards you")

    euler_relaxed_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)

    reg_shoulder = linear_model.LinearRegression()
    reg_shoulder.fit([euler_relaxed_12, euler_side_12, euler_fist_up_12, euler_front_12, euler_arm_up_12], [relaxed_values, side_values, fist_up_values, front_values, arm_up_values])


    #reg_elbow = linear_model.LinearRegression()
    #eg_elbow.fit([euler_front_23, euler_bend_23], [0, 1.57])

    
    csvfile =  open('calibration.csv', 'w')
    writer = csv.writer(csvfile)

    print (reg_shoulder.coef_[0])
    print ([0,0,0])
    print (reg_shoulder.intercept_[0])
    writer.writerow (np.concatenate((reg_shoulder.coef_[0], np.array([0,0,0]), np.array([reg_shoulder.intercept_[0]]))))
    writer.writerow (np.concatenate((reg_shoulder.coef_[1], np.array([0,0,0]), np.array([reg_shoulder.intercept_[1]]))))
    writer.writerow (np.concatenate((reg_shoulder.coef_[2], np.array([0,0,0]), np.array([reg_shoulder.intercept_[2]]))))

    #writer.writerow ([0,0,0] + reg_elbow.coef_ + [reg_elbow.intercept_])