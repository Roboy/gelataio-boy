#!/usr/bin/env python

from scooping_cv.srv import *
import rospy

def handle_goal_req(req):
	if(req.goal == 0):
		# Scooping
		pass
	else if(req.goal == 1):
		# Serving the scoop to the cup
		pass

	x = 1.51
	y = 2.32
	z = 0.5
    print "Returning [%s + %s = %s]"%(x, y, z)
    return x,y,z

def compute_goal_server():
    rospy.init_node('goal_pose_server')
    s = rospy.Service('goal_pose', GoalPose, handle_goal_req)
    print "Ready to send back your next mission."
    rospy.spin()

if __name__ == "__main__":
    compute_goal_server()
