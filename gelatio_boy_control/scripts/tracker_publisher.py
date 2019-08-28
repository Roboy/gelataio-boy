#!/usr/bin/env

import triad_openvr
import time
import rospy
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

    coefficients = []
    csvfile =  open('calibration.csv', 'rb')
    reader = csv.reader(csvfile)

    for row in reader:
        coefficients.append(row)


    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()

    interval = 1/10

    initial_pose1 = v.devices["tracker_1"].get_pose_quaternion()
    initial_pose2 = v.devices["tracker_2"].get_pose_quaternion()
    initial_pose3 = v.devices["tracker_3"].get_pose_quaternion()

    joint_state = rospy.Publisher('/joint_states', sensor_msgs.msg.JointState , queue_size=1)
    shoulder_right_axis0_publisher = rospy.Publisher('/shoulder_right_axis0/shoulder_right_axis0/target', std_msgs.msg.Float32 , queue_size=1)
    shoulder_right_axis1_publisher = rospy.Publisher('/shoulder_right_axis1/shoulder_right_axis1/target', std_msgs.msg.Float32 , queue_size=1)
    shoulder_right_axis2_publisher = rospy.Publisher('/shoulder_right_axis2/shoulder_right_axis2/target', std_msgs.msg.Float32 , queue_size=1)

    elbow_right_publisher = rospy.Publisher('elbow_right/elbow_right/target', std_msgs.msg.Float32 , queue_size=1)


    while not rospy.is_shutdown():
        start = time.time()

        euler_12, euler_23 = _get_instance_eulers(initial_pose1, initial_pose2, initial_pose3)

        """
        msg = sensor_msgs.msg.JointState()
        msg.header = std_msgs.msg.Header()
        msg.header.stamp = rospy.Time.now()
        msg.name = ['sphere_axis0', 'sphere_axis1', 'sphere_axis2']
        """


        print ("Shoulder angles: ", euler_12)
        print ("Elbow angles: ", euler_23)

        shoulder_axis0 = coefficients[0][0]*euler_12[0] + coefficients[0][1]*euler_12[1] + coefficients[0][2]*euler_12[2] + coefficients[0][6]
        shoulder_axis1 = coefficients[1][0]*euler_12[0] + coefficients[1][1]*euler_12[1] + coefficients[1][2]*euler_12[2] + coefficients[1][6]
        shoulder_axis2 = coefficients[2][0]*euler_12[0] + coefficients[2][1]*euler_12[1] + coefficients[2][2]*euler_12[2] + coefficients[2][6]
        elbow_axis = coefficients[3][3]*euler_12[0] + coefficients[3][4]*euler_12[1] + coefficients[3][5]*euler_12[2] + coefficients[3][6]

        shoulder_right_axis0_publisher.publish(std_msgs.msg.Float32(shoulder_axis0))
        shoulder_right_axis1_publisher.publish(std_msgs.msg.Float32(shoulder_axis1))
        shoulder_right_axis2_publisher.publish(std_msgs.msg.Float32(shoulder_axis2))

        elbow_right_publisher.publish(std_msgs.msg.Float32(elbow_axis))

