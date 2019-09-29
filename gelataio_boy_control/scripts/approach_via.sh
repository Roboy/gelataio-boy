#!/bin/bash
rostopic pub /joint_targets sensor_msgs/JointState "{name: [shoulder_right_axis0, shoulder_right_axis1, shoulder_right_axis2, elbow_right], position:[-1.3, 0.8, 1.7, 1.8], velocity:[0.0]}" -1
