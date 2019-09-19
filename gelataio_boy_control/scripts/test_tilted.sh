#!/bin/bash
rostopic pub /joint_targets sensor_msgs/JointState "{name: [shoulder_right_axis0, shoulder_right_axis1, shoulder_right_axis2, elbow_right], position:[-1, 0.5, 1.45, 1.02], velocity:[0.0]}" -1 &
