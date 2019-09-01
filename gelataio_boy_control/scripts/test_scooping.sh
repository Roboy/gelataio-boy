#!/bin/bash
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.55, z: 0.35}}'
rostopic echo /scooping_planning/status
