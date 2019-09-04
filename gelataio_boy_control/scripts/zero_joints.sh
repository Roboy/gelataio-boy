#!/bin/bash
rostopic pub /joint_targets sensor_msgs/JointState "{name: [shoulder_right_axis0, shoulder_right_axis1, shoulder_right_axis2, elbow_right], position:[0, 0.2, 0, 0.0], velocity:[0.0]}" -1 &
rostopic pub -1 /roboy/middleware/MotorCommand roboy_middleware_msgs/MotorCommand "{id: 6, motors: [2], set_points: [0]}" &
