#!/bin/bash
rosservice call /scooping_planning/scoop '{start_point: {x: 0, y: -0.5, z: 0.35}}'
rostopic echo /scooping_planning/status
