#!/bin/bash
rosservice call /scooping_planning/init_pose
rosservice call /scooping_planning/scoop '{start_point: {x: 0.0, y: -0.55, z: 0.18}}'
rostopic echo /scooping_planning/status
rosservice call /scooping_planning/init_pose
