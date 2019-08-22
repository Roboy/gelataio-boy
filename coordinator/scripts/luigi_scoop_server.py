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

from gelataio_msgs.srv import PerformScoop
from geometry_msgs.msg import Pose

import numpy as np

class ScoopServer:
  _feedback = OrderIceCreamFeedback()
  _result = OrderIceCreamResult()

  def __init__(self):
    # scoops = []
    # flavors = []
    self._action_name = 'luigi_scoop'
    self.doneScooping_ = False
    self.scooping_status_ = 'Did not recieve any ice cream order yet!'

    callback_lambda = lambda x: self.RecieveIceCreamOrder_(x)
    self.server_ = actionlib.SimpleActionServer('luigi_scoop', OrderIceCreamAction, execute_cb=callback_lambda , auto_start=False)
    self.server_.start()

  # Callback for sub of topic "scooping_planning/status"
  def ScoopStatusCallback(self, scooping_status, args):
    self.scooping_status_ = scooping_status
    print(scooping_status)


  # Arguments are of Pose type
  def PerformScoopClient(self, startPose, endPose):
    rospy.wait_for_service('scooping_planning/scoop')
    try:
      PerformScoopServClient = rospy.ServiceProxy('scooping_planning/scoop', PerformScoop)
      resp = PerformScoopServClient(x, y)
      return resp.sum
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e


  # string[], int32[]
  def RecieveIceCreamOrder_(self, data):

    self.scooping_status_ = 'Recived a new ice cream order'
    scoops = data.scoops
    flavors = data.flavors

    r = rospy.Rate(1)
    success = True

    # publish info to the console for the user
    # rospy.loginfo('%s: Recieved, Ice cream order of %i scoops' % 
    #   (self._action_name, scoopsNumber))

    # start executing the action
    # check that preempt has not been requested by the client
    if self.server_.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self.server_.set_preempted()
      success = False

 
    self._feedback.finished_scoops = []
    for i in scoops:
      self._feedback.finished_scoops.append(0)
    self._feedback.status_message = self.scooping_status_
    # publish the feedback
 
    self.server_.publish_feedback(self._feedback)

    # We don't want all surfaces, just the flavour that we want to scoop for now

    # Waiting for sufaces from vision
    for flavor in flavors:
        for scoop in scoops:
            pass
        pass

    # Request Matrix from vision
    #

    # For now sdoneScooping_ is set manually by trial and error
    # Can vision confirm if the scoop has been filled? for now, No
    while not self.doneScooping_:
      
      # TODO: The scooper is on the top of the icecream box
      # TODO: The scooper always starts at the top of the icecream box


      # Fancy scooping algorthim
      #
      for i in range(10):
        # Calculate points
        # Waiting for the surface class
        pass

      # Use a dummy matrix for now
      # Go to the highest point near the corner
      # TODO: as Alona mentioned we have no idea what are the 
      # cpablities of the hand righ now
      # We only go horizantly (along the longest axis of the icecream box)
      # surface = np.random.array((30, 20, 5))
      # for i in surface.shape[0]:
      #   for j in surface.shape[1]:
      #     print(surface[i][j])
 
      # call scooping service with the points
      #
      startPose = Pose()
      startPose.position.x = 0.5
      startPose.position.y = -0.1
      startPose.position.z = 1.0
      # Make sure the quaternion is valid and normalized
      startPose.orientation.x = 0.0
      startPose.orientation.x = 0.0
      startPose.orientation.x = 0.0
      startPose.orientation.w = 1.0

      endPose = Pose()
      endPose.position.x = 0.5
      endPose.position.y = -0.1
      endPose.position.z = 1.0
      # Make sure the quaternion is valid and normalized
      endPose.orientation.x = 0.0
      endPose.orientation.x = 0.0
      endPose.orientation.x = 0.0
      endPose.orientation.w = 1.0

      # TODO: after the scooping service is up uncomment this
      # scoopingRespose = self.PerformScoopClient(startPose, endPose)


      # TODO: test that scooping is acctually finished which for now should be just set manually
      # after certain number to times
      if self.scooping_status_ == 'Done':
        # TODO: remove after testing
        self.doneScooping_ = True

      self.doneScooping_ = True

      break

    print(success)
    # Send result
    if success:
      self._result.success = True
      self._result.error_message = 'Failed to come up with a plan'
      self.server_.set_succeeded(self._result)


if __name__ == '__main__':
  rospy.init_node('luigi_scoop_server')
  server = ScoopServer()
  rospy.spin()
  try:
    rospy.Subscriber("scooping_planning/status", String, server.ScoopStatusCallback, 1)
  except KeyboardInterrupt, e:
    pass