#! /usr/bin/env python

import roslib
roslib.load_manifest('coordinator')
import rospy
import actionlib
from std_msgs.msg import *
from std_srvs.srv import Trigger, TriggerRequest
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
scoopingSteps = 1

class ScoopServer:
  _feedback = OrderIceCreamFeedback()
  _result = OrderIceCreamResult()
  _feedback_delay = 5

  def __init__(self):
    self.we_have_client_ = False
    self._action_name = 'luigi_scoop'
    self.doneScooping_ = False
    self._feedback.finished_scoops = []

    # Status as to be sent for luigig as a feedback
    # This is always overriden to "more time" as requested from luigi
    self.scooping_human_status_ = 'Did not receive any ice cream order yet!'
    
    # Status as recived from scooping_planning/scoop
    self.scooping_status_ = String

    callback_lambda = lambda x: self.ReceiveIceCreamOrder_(x)
    self.server_ = actionlib.SimpleActionServer('luigi_scoop', OrderIceCreamAction, callback_lambda, auto_start=False)
    self.server_.start()

    # Scooping callback lamda
    self.scooping_callback_lambda_ = lambda x: self.ScoopStatusCallback(x)

  def ScoopStatusCallback(self, scooping_status):
    self.scooping_status_ = scooping_status

  # Arguments are of Pose type
  def GoHome(self):
    rospy.wait_for_service('scooping_planning/go_home')
    try:
      # TODO: Only call this when status is IDLE
      goHome = rospy.ServiceProxy('scooping_planning/go_home', Trigger)

      # Create an object of the type TriggerRequest. We nned a TriggerRequest for a Trigger service
      req = TriggerRequest()

      # Now send the request through the connection
      resp = goHome(req)
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

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

  # For now we set which flavor is where by hand (manually), unless vision can provide us with an api
  def GetFlavorStartingPoint(self, flavor):
    leftStartPoint = Point(x=-0.27, y=-0.5, z=0.6)
    rightStartPoint = Point(x=-0.27, y=-0.5, z=0.6)
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

    self.we_have_client_ = True

    # success of the total scoping operation
    success = False

    # Scoops required
    scoops = data.scoops

    # flavors for each "scoops" ordered, not scoop :D
    flavors = data.flavors

    self.scooping_human_status_ = 'Received a new ice cream order'
    # intially no scoops has been scooped
    for i in scoops:
      self._feedback.finished_scoops.append(0)

    scoops = data.scoops
    flavors = data.flavors

    # start executing the action
    # check that preempt has not been requested by the client
    if self.server_.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self.server_.set_preempted()
      success = False

    # success of the total scoping operation
    success = True

    # The rate by which we check the scooping status change
    r = rospy.Rate(3) # 3hz

    # We try to scoop in just five points sequence
    for scoop in range(len(scoops)):

      # Go to the start point of the box, depending on the flavor
      startingPoint = self.GetFlavorStartingPoint(flavors[scoop])

      # Loop for how many scoops of this flavor, really luigi??
      for scoopPerFlavor in range(scoops[scoop]):

        # # Go to start point depending on the flavor
        # scoopingResponse = self.TranslationalPTPMotionClient(startingPoint, startingPoint)
        # if not scoopingResponse:
        #   self.scooping_human_status_ = 'Failed to come up with a plan for step ' + str(i) + ' out of 5'
        success = True

        # Each scoop is done in scoopingSteps
        for i in range(scoopingSteps):
          # Wait for scooping_planning/status to be idle
          # Calculate new point
          # Send point to scooping_planning/scoop

          # Wait for scooping_planning/status to be idle
          while not str(self.scooping_status_.data) == 'IDLE' and not rospy.is_shutdown():
            self.scooping_human_status_ = str(self.scooping_status_.data) + ' need more time'
            rospy.sleep(1.)

          # If not IDLE, update scooping status
          self.scooping_human_status_ = str(self.scooping_status_.data) + ' in step ' + str(i) + ' out of 5'

          # Now for step 2 and 3
          # Move along the y-z plane (ice cream surface)     
          scoopingResponse = self.TranslationalPTPMotionClient(startingPoint, startingPoint)
          if not scoopingResponse:
            self.scooping_human_status_ = 'Failed to come up with a plan for step ' + str(i) + ' out of 5'
            success = False
          else:
            success = True

          r.sleep()
          

          # Wait for scooping_planning/status to be idle
          while not str(self.scooping_status_.data) == 'IDLE' and not rospy.is_shutdown():
            self.scooping_human_status_ = str(self.scooping_status_.data) + ' need more time'
            rospy.sleep(1.)          

          # Has to sleep here because the rate we check for the scooping status is faster than
          # the scooping status publish rate, now sleep rate is 3hz, scoop publish rate is 5hz
          rospy.sleep(1)
               
          #   # call scooping service with the points
          #   #
      self._feedback.finished_scoops[scoop] = 1

    while not str(self.scooping_status_.data) == 'IDLE':
      rospy.loginfo('Waiting for scooping to go idle')
      rospy.sleep(1.)

    wentHome = self.GoHome()

    if wentHome:
      rospy.loginfo('Went home')
    else:
      rospy.logwarn('Could not go home, the traffic is terrible')

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
    if self.we_have_client_: 
      self._feedback.status_message = self.scooping_human_status_

      self._feedback.status_message = "more time"
      self.server_.publish_feedback(self._feedback)
      s.enter(self._feedback_delay, 1, self.SendFeedbackLuigi, (sc,))
    else:
      rospy.loginfo('Luigi is Dead, move on!')
      s.enter(self._feedback_delay, 1, self.SendFeedbackLuigi, (sc,))

if __name__ == '__main__':
  
  # Schedule feedback
  s = sched.scheduler(time.time, time.sleep)
  rospy.init_node('luigi_scoop_server')
  server = ScoopServer()
  try:
    rospy.Subscriber("scooping_planning/status", String, server.ScoopStatusCallback)
  except KeyboardInterrupt, e:
    pass

  s.enter(server._feedback_delay, 1, server.SendFeedbackLuigi, (s,))
  s.run()
  rospy.spin()
