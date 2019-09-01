#!/bin/bash
rostopic pub /joint_targets sensor_msgs/JointState "{name: [ice_left_linear, ice_right_linear], position:[0.25, 0.25], velocity:[0.0, 0.0]}"
