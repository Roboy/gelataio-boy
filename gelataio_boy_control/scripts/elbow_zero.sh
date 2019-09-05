#!/bin/bash
rostopic pub /joint_targets sensor_msgs/JointState "{name: [elbow_right], position:[1.57], velocity:[0.0]}" -1
