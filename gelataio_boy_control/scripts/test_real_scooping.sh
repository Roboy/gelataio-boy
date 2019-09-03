#!/bin/bash
# rosservice call /scooping_planning/init_pose
bash approach_via.sh
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.6, z: 0.45}}'
rostopic echo /scooping_planning/status
