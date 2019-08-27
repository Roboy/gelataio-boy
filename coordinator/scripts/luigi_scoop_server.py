#! /usr/bin/env python

import roslib
roslib.load_manifest('coordinator')
import rospy
import actionlib
from std_msgs.msg import *
import sched, time

# Imported msgs are OrderIceCreamAction, OrderIceCreamActionFeedback
# OrderIceCreamActionGoal, OrderIceCreamActionResult, OrderIceCreamFeedback 
# OrderIceCreamGoal, OrderIceCreamResult
from roboy_cognition_msgs.msg import *

from roboy_control_msgs.srv import TranslationalPTPMotion
from geometry_msgs.msg import Point

import numpy as np

# First one is left box, second one is right box
availableFlavors = ['choco', 'spagetti']

# The number of steps we need to scoop a single scoop (tuned manually)
scoopingSteps = 5

class ScoopServer:
  _feedback = OrderIceCreamFeedback()
  _result = OrderIceCreamResult()
  _feedback_delay = 5

  def __init__(self):
    self._action_name = 'luigi_scoop'
    self.doneScooping_ = False
    self._feedback.finished_scoops = []

    # Status as to be sent for luigig as a feedback
    self.scooping_human_status_ = 'Did not receive any ice cream order yet!'
    
    # Status as recived from scooping_planning/scoop
    self.scooping_status_ = 'NULL'

    callback_lambda = lambda x: self.ReceiveIceCreamOrder_(x)
    self.server_ = actionlib.SimpleActionServer('luigi_scoop', OrderIceCreamAction, callback_lambda, auto_start=False)
    self.server_.start()

  # Callback for sub of topic "scooping_planning/status"
  def ScoopStatusCallback(self, scooping_status):
    self.scooping_status_ = scooping_status
    print(scooping_status)

  # Arguments are of Pose type
  def TranslationalPTPMotionClient(self, startPosition, endPosition):
    rospy.wait_for_service('scooping_planning/scoop')
    try:
      # TODO: Only call this when status is IDLE
      TranslationalPTPMotionServClient = rospy.ServiceProxy('scooping_planning/scoop', TranslationalPTPMotion)
      resp = TranslationalPTPMotionServClient(startPosition, endPosition)
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  def SendFeedbackLuigi(self, feedback):
    for i in scoops:
      self._feedback.finished_scoops.append(0)
    self._feedback.status_message = 'This is feedback'
    # publish the feedback 
    self.server_.publish_feedback(self._feedback)


  # For now we set which flavor is where by hand (manually), unless vision can provide us with an api
  def GetFlavorStartingPoint(self, flavor):
    leftStartPoint = Point(x=-0.1, y=-0.4, z=0.25)
    rightStartPoint = Point(x=-0.1, y=-0.4, z=0.25)
    startingPoint = Point()

    # Left ice cream box
    if flavor == availableFlavors[0]:
      startingPoint = leftStartPoint
    elif flavor == availableFlavors[1]:
      startingPoint = rightStartPoint
    # Do we fancy a third ice cream box?
    else:
      startingPoint = leftStartPoint

    return startingPoint

    
  def ReceiveIceCreamOrder_(self, data):

    # success of the total scoping operation
    success = False

    # Scoops required
    scoops = data.scoops

    # Scoops done
    scoopsScooped = np.zeros(len(scoops))

    # flavors for each "scoops" ordered, not scoop :D
    flavors = data.flavors

    self.scooping_human_status_ = 'Recived a new ice cream order'
    # intially no scoops has been scooped
    for i in scoops:
      self._feedback.finished_scoops.append(0)

    scoops = data.scoops
    flavors = data.flavors

    r = rospy.Rate(1)

    # start executing the action
    # check that preempt has not been requested by the client
    if self.server_.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self.server_.set_preempted()
      success = False

    # success of the total scoping operation
    success = True
    # We try to scoop in just five points sequence
    for scoop in range(len(scoops)):

      # Go to the start point of the box, depending on the flavor
      startingPoint = self.GetFlavorStartingPoint(flavors[scoop])

      # Loop for how many scoops of this flavor, really luigi??
      for scoopPerFlavor in range(scoops[scoop]):

        # Go to start point depending on the flavor
        scoopingResponse = self.TranslationalPTPMotionClient(startingPoint, startingPoint)
        if not scoopingResponse:
          self.scooping_human_status_ = 'Failed to come up with a plan for step ' + str(i) + ' out of 5'
          success = False

        # Each scoop is done in scoopingSteps
        for i in range(scoopingSteps):
          # Wait for scooping_planning/status to be idle
          # Calculate new point
          # Send point to scooping_planning/scoop

          # Wait for scooping_planning/status to be idle
          while not self.scooping_status_ == 'IDLE':
            self.scooping_human_status_ = self.scooping_status_ + ' need more time'
            rospy.sleep(1.)

          # If not IDLE, update scooping status
          self.scooping_human_status_ = self.scooping_status_ + ' in step ' + str(i) + ' out of 5'


          # Now for step 2 and 3
          # Move along the y-z plane (ice cream surface)     
          scoopingResponse = self.TranslationalPTPMotionClient(Point(x=-0.1, y=-0.4, z=0.25), Point(x=-0.1, y=-0.3, z=0.25))
          if not scoopingResponse:
            self.scooping_human_status_ = 'Failed to come up with a plan for step ' + str(i) + ' out of 5'
            success = False

          # Fill up self._feedback.scoops[] with 1s for every scoop scooped

          # For now sdoneScooping_ is set manually by trial and error
          # Can vision confirm if the scoop has been filled? for now, No
          # while not self.doneScooping_:
          
          # TODO: The scooper is on the top of the icecream box
          # TODO: The scooper always starts at the top of the icecream box

          # Use a dummy matrix for now
          # Go to the highest point near the corner
          # TODO: as Alona mentioned we have no idea what are the 
          # cpablities of the hand righ now
          # We only go horizantly (along the longest axis of the icecream box)
          # surface = np.random.array((30, 20, 5))
          # for i in surface.shape[0]:
          #   for j in surface.shape[1]:
          #     print(surface[i][j])
     
        #   # call scooping service with the points
        #   #

    if success:
      self._result.success = True
      self._result.error_message = self.scooping_human_status_
      self.server_.set_succeeded(self._result)      

    # Send result
    if not success:
      self._result.success = False
      self._result.error_message = self.scooping_human_status_
      self.server_.set_succeeded(self._result)

  # This function is called every _feedback_delay seconds
  def SendFeedbackLuigi(self, sc): 
    self._feedback.status_message = self.scooping_human_status_
    self.server_.publish_feedback(self._feedback)
    s.enter(self._feedback_delay, 1, self.SendFeedbackLuigi, (sc,))


if __name__ == '__main__':
  
  # Schedule feedback
  s = sched.scheduler(time.time, time.sleep)
  rospy.init_node('luigi_scoop_server')
  server = ScoopServer()
  try:
    rospy.Subscriber("scooping_planning/status", String, server.ScoopStatusCallback, 1)
  except KeyboardInterrupt, e:
    pass

  s.enter(server._feedback_delay, 1, server.SendFeedbackLuigi, (s,))
  s.run()
  rospy.spin()
