#!/usr/bin/env python

import sys
import rospy
from scooping_cv.srv import *

def get_next_point(x, y):
    rospy.wait_for_service('goal_pose')
    try:
        next_point = rospy.ServiceProxy('goal_pose', GoalPose)
        resp1 = next_point(x)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        x = str(sys.argv[1])
    else:
        print usage()
        sys.exit(1)
    print "Next point to reach %s"%(x)
    print "%s -> [%s, %s, %s]"%(x, get_next_point(x))