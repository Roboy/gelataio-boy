#! /usr/bin/env python

import roslib
roslib.load_manifest('coordinator')
import rospy
import actionlib
from std_msgs.msg import *

# Imported msgs are OrderIceCreamAction, OrderIceCreamActionFeedback
# OrderIceCreamActionGoal, OrderIceCreamActionResult, OrderIceCreamFeedback 
# OrderIceCreamGoal, OrderIceCreamResult
from roboy_cognition_msgs.msg import *

def scoopingFeeback(feedback):
  print('Received some feed back:')
  print(feedback)

def luigi_scoop_client(x, y):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('luigi_scoop', OrderIceCreamAction)

    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = OrderIceCreamGoal(flavors=x, scoops=y)

    # Sends the goal to the action server.
    client.send_goal(goal, feedback_cb=scoopingFeeback)
    # client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('luigi_scoop_client')
        flavors = ['blue', 'red', 'spagitti lemon'] 
        scoops = [1, 2, 1]
        result = luigi_scoop_client(flavors, scoops)
        print(result)
        # print("Result:", ', '.join([str(n) for n in result]))
    except rospy.ROSInterruptException:
        print("program interrupted before completion")
