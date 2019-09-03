#!/bin/bash
rosservice call /scooping_planning/init_pose
rosservice call /scooping_planning/scoop '{start_point: {x: 0.0, y: -0.45, z: 0.30}}'
rostopic echo /scooping_planning/status
./zero_joints.sh
