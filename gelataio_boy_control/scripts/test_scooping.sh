#!/bin/bash
rosservice call /scooping_planning/scoop '{start_point: {x: -0.1, y: -0.4, z: 0.3}}'
rostopic echo /scooping_planning/status
