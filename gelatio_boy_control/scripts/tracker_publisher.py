#!/usr/bin/env python3

import triad_openvr
import time
import rospy
import numpy as np
import math
from pyquaternion import Quaternion
import std_msgs
from sensor_msgs.msg import JointState
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
    torso_tracker_name = "tracker_2"
    shoulder_tracker_name = "tracker_3"


    coefficients = []
    csvfile =  open('calibration.csv', 'r')
    reader = csv.reader(csvfile)

    for row in reader:
        r = np.array([float(i) for i in row])
        coefficients.append(r)


    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()


    initial_pose_torso = v.devices[torso_tracker_name].get_pose_quaternion()
    initial_pose_shoulder = v.devices[shoulder_tracker_name].get_pose_quaternion()

    joint_target_publisher = rospy.Publisher('/joint_targets', JointState, queue_size=1)


    #elbow_right_publisher = rospy.Publisher('elbow_right/elbow_right/target', std_msgs.msg.Float32 , queue_size=1)


    while not rospy.is_shutdown():
        start = time.time()

        euler_12 = _get_euler_angles_difference(torso_tracker_name, shoulder_tracker_name, initial_pose_torso, initial_pose_shoulder)



        print ("Shoulder angles: ", euler_12)
        #print ("Elbow angles: ", euler_23)


        shoulder_axis0 = coefficients[0][0]*euler_12[0] + coefficients[0][1]*euler_12[1] + coefficients[0][2]*euler_12[2] + coefficients[0][6]
        shoulder_axis1 = coefficients[1][0]*euler_12[0] + coefficients[1][1]*euler_12[1] + coefficients[1][2]*euler_12[2] + coefficients[1][6]
        shoulder_axis2 = coefficients[2][0]*euler_12[0] + coefficients[2][1]*euler_12[1] + coefficients[2][2]*euler_12[2] + coefficients[2][6]
        print ([shoulder_axis0, shoulder_axis1, shoulder_axis2])
        #elbow_axis = coefficients[3][3]*euler_12[0] + coefficients[3][4]*euler_12[1] + coefficients[3][5]*euler_12[2] + coefficients[3][6]


        state = JointState(std_msgs.msg.Header(),["shoulder_right_axis0", "shoulder_right_axis1","shoulder_right_axis2"],
                                           [shoulder_axis0, shoulder_axis1, shoulder_axis2],
                                           [0,0,0],
                                           [0,0,0])

        joint_target_publisher.publish(state)
        #shoulder_right_axis0_publisher.publish(std_msgs.msg.Float32(shoulder_axis0))
        #shoulder_right_axis1_publisher.publish(std_msgs.msg.Float32(shoulder_axis1))
        #shoulder_right_axis2_publisher.publish(std_msgs.msg.Float32(shoulder_axis2))

        #elbow_right_publisher.publish(std_msgs.msg.Float32(elbow_axis))

