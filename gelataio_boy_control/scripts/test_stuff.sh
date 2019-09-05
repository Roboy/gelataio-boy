#!/bin/bash
rosservice call /scooping_planning/init_pose
sleep 4.0
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.43, z: 0.31}}'
