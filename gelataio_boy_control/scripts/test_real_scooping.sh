#!/bin/bash
rosservice call /scooping_planning/init_pose
sleep 5.0
rostopic pub /roboy/cognition/face/show_emotion std_msgs/String "lucky" -1 & 
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.43, z: 0.31}}'
rostopic pub /roboy/cognition/face/show_emotion std_msgs/String "surprise" -1 & 
rostopic echo /scooping_planning/status
#rosservice call /scooping_planning/init_pose
bash approach_via.sh
sleep 3.0
rostopic pub /joint_targets sensor_msgs/JointState "{name: [shoulder_right_axis0, shoulder_right_axis1, shoulder_right_axis2, elbow_right], position:[-1.4, 0.6, 1.5, 0.8], velocity:[0.0]}" -1
sleep 5.0
bash wrist_descoop.sh
rostopic pub /roboy/cognition/face/show_emotion std_msgs/String "hypno_color" -1 & 

