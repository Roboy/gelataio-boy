#!/usr/bin/env

import triad_openvr
import time
import rospy
import tf
import numpy as np
import math
from pyquaternion import Quaternion
import std_msgs, sensor_msgs
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


def process_transform(device_name, pose ):
    try:
        pose = v.devices[device_name].get_pose_quaternion()
    except:
        rospy.loginfo("could not find transform world->"+ device_name +", initialization might be wrong")

    q_tracker = Quaternion(pose[6],pose[3],pose[4],pose[5])                 #*q_init1.inverse
    return q_tracker

def _get_instance_eulers():
    q_tracker_1 = process_transform("tracker_1", initial_pose1)
    q_tracker_2 = process_transform("tracker_3", initial_pose2)
    q_tracker_3 = process_transform("tracker_2", initial_pose3)

    q_tracker_diff_12 = q_tracker_2*q_tracker_1.inverse
    q_tracker_diff_23 = q_tracker_3*q_tracker_2.inverse

    euler_12 = rotationMatrixToEulerAngles(q_tracker_diff_12.rotation_matrix)
    euler_23 = rotationMatrixToEulerAngles(q_tracker_diff_23.rotation_matrix)

    return euler_12, euler_23


if __name__ == "__main__":
    # use these to change publishing behaviour
    head = False
    shoulder_left = True

    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()

    interval = 1/10

    initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
    initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
    initial_pose3 = v.devices["tracker_3"].get_pose_quaternion()

    q_init1 = Quaternion(initial_pose1[6],initial_pose1[3],initial_pose1[4],initial_pose1[5])
    q_init2 = Quaternion(initial_pose2[6],initial_pose2[3],initial_pose2[4],initial_pose2[5])
    q_init3 = Quaternion(initial_pose3[6],initial_pose3[3],initial_pose3[4],initial_pose3[5])

    X0 = np.array([1,0,0])
    X1 = np.array([0,1,0])
    X2 = np.array([0,0,1])
    trans_top = np.array([0,0,0])

    align_to_world = Quaternion([0,0,0,1])

    q_tracker_1 = process_transform("tracker_1", initial_pose1)
    q_tracker_2 = process_transform("tracker_3", initial_pose2)
    q_tracker_3 = process_transform("tracker_2", initial_pose3)

    q_tracker_diff_12 = q_tracker_2*q_tracker_1.inverse
    q_tracker_diff_23 = q_tracker_3*q_tracker_2.inverse

    tracker_diff_12 = q_tracker_diff_12.rotation_matrix
    tracker_diff_23 = q_tracker_diff_23.rotation_matrix

    Y0_12 = np.array(tracker_diff_12[0][:])
    Y1_12 = np.array(tracker_diff_12[1][:])
    Y2_12 = np.array(tracker_diff_12[2][:])

    Y0_23 = np.array(tracker_diff_23[0][:])
    Y1_23 = np.array(tracker_diff_23[1][:])
    Y2_23 = np.array(tracker_diff_23[2][:])

    rot_align_12 = np.array([[X0.dot(Y0_12),X0.dot(Y1_12),X0.dot(Y2_12)],[X1.dot(Y0_12),X1.dot(Y1_12),X1.dot(Y2_12)],[X2.dot(Y0_12),X2.dot(Y1_12),X2.dot(Y2_12)]])
    rot_align_23 = np.array([[X0.dot(Y0_23),X0.dot(Y1_23),X0.dot(Y2_23)],[X1.dot(Y0_23),X1.dot(Y1_23),X1.dot(Y2_23)],[X2.dot(Y0_23),X2.dot(Y1_23),X2.dot(Y2_23)]])


    input("Relax your arm")

    euler_relaxed_12, euler_relaxed_23 = _get_instance_eulers()