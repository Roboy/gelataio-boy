#!/usr/bin/env

import triad_openvr
import time
import rospy
import tf
import numpy as np
import math
from sklearn import linear_model
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

def _get_instance_eulers(initial_pose1, initial_pose2, initial_pose3):
    q_tracker_1 = process_transform("tracker_1", initial_pose1)
    q_tracker_2 = process_transform("tracker_2", initial_pose2)
    q_tracker_3 = process_transform("tracker_3", initial_pose3)

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

    #values for cardsflow

    relaxed_values = [-0.625, -0.436, -0.383]

    side_values = [0.07, 1.011, -0.628]

    front_values = [1.064, 0.314, 0.436]

    input("Relax your arm, put it down parallel to your torso with the palm facing towards you")

    initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
    initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
    initial_pose3 = v.devices["tracker_3"].get_pose_quaternion()

    input("Point your right arm to the right, orthogonal to your body and parallel to the ground with your palm facing down")

    euler_side_12, _ = _get_instance_eulers(initial_pose1, initial_pose2, initial_pose3)

    input("Point your right arm infront of you, orthogonal to your body and parallel to the ground, with your palm facing left")

    euler_front_12, euler_front_23 = _get_instance_eulers(initial_pose1, initial_pose2, initial_pose3)

    input("Now, keeping the same position, bend your elbow 90 degrees to now your hand faces your body")

    _, euler_bend_23 = _get_instance_eulers(initial_pose1, initial_pose2, initial_pose3)

    reg_shoulder = linear_model.LinearRegression()
    reg_shoulder.fit([[0,0,0], euler_side_12, euler_front_12], [relaxed_values, side_values, front_values])

      
    reg_elbow = linear_model.LinearRegression()
    reg_elbow.fit([euler_front_23, euler_bend_23], [0, 1.57])

    
    csvfile =  open('calibration.csv', 'w')
    writer = csv.writer(csvfile)

    writer.writerow (reg_shoulder.coef_[0] + [0,0,0] + [reg_shoulder.intercept_[0]])
    writer.writerow (reg_shoulder.coef_[1] + [0,0,0] + [reg_shoulder.intercept_[1]])
    writer.writerow (reg_shoulder.coef_[2] + [0,0,0] + [reg_shoulder.intercept_[2]])

    writer.writerow ([0,0,0] + reg_elbow.coef_ + [reg_elbow.intercept_])