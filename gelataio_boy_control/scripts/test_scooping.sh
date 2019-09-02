#!/bin/bash
bash appraoch_via.sh
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.6, z: 0.4}}'
rostopic echo /scooping_planning/status
