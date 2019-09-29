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
from roboy_cognition_msgs.srv import DetectIceCream

from roboy_control_msgs.srv import TranslationalPTPMotion
from geometry_msgs.msg import Point, PointStamped
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
  def GetPoint(self):
    print('getPoint')
    rospy.wait_for_service('/iceCreamService')
    try:
      getPoint = rospy.ServiceProxy('/iceCreamService', DetectIceCream)
      req = 'flakes'
      print("WE HAVE A POINT")
      # Now send the request through the connection
      resp = getPoint(req)
      print('getPoint::response')
      return resp.start_scooping
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

  # Arguments are of Pose type
  def InitPose(self):
    rospy.wait_for_service('scooping_planning/init_pose')
    try:
      # TODO: Only call this when status is IDLE
      init_pose = rospy.ServiceProxy('scooping_planning/init_pose', Trigger)
      resp = init_pose()
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

  # For now we set which flavor is where by hand (manually), unless vision can provide us with an api
  def GetFlavorStartingPoint(self, flavor):
    leftStartPoint = Point(x=0.0, y=-0.55, z=0.18)
    rightStartPoint = Point(x=0.0, y=-0.55, z=0.18)
    startingPoint = Point()

    # Left ice cream box
    if flavor == availableFlavors[0]:
      startingPoint = leftStartPoint
    elif flavor == availableFlavors[1]:
      startingPoint = rightStartPoint
    # Do we fancy a third ice cream box?
    else:
      startingPoint = leftStartPoint

    startingPoint = self.GetPoint()
    return startingPoint

    
  def ReceiveIceCreamOrder_(self, data):

    self._feedback.finished_scoops = []
    self.we_have_client_ = True

    rospy.loginfo(data)
    # success of the total scoping operation
    success = False
    # Scoops required
    scoops = data.scoops
    # flavors for each "scoops" ordered, not scoop :D
    flavors = data.flavors
    self.scooping_human_status_ = 'Received a new ice cream order'
    # intially no scoops has been scooped
    for i in range(len(scoops)):
      self._feedback.finished_scoops.append(0)

    # start executing the action
    # check that preempt has not been requested by the client
    if self.server_.is_preempt_requested():
      rospy.loginfo('%s: Preempted' % self._action_name)
      self.server_.set_preempted()
      success = False

    # success of the total scoping operation
    success = False

    # The rate by which we check the scooping status change
    r = rospy.Rate(5) # 5hz
    busy = rospy.Rate(0.5)

    # Call init service
    init_result = self.InitPose()

    if not init_result:
      rospy.loginfo('Cant go to init')
      # rospy.shutdown()

    # sleep for 5 seconds, waiting for init
    rospy.sleep(5.)

    # We try to scoop in just five points sequence
    for scoop in range(len(scoops)):

      # Go to the start point of the box, depending on the flavor
      startingPoint = self.GetFlavorStartingPoint(flavors[scoop]).point

      print(startingPoint)
      # Loop for how many scoops of this flavor, really luigi??
      for scoopPerFlavor in range(scoops[scoop]):
        print('Scoops per flavor ', scoops[scoop])
        # Each scoop is done in scoopingSteps
        # Wait for scooping_planning/status to be idle
        while not str(self.scooping_status_.data) == 'DONE' and not str(self.scooping_status_.data) == 'FAIL' and not rospy.is_shutdown():
          # excuting or planning
          r.sleep()

        scoopingResponse = self.TranslationalPTPMotionClient(startingPoint, startingPoint)
        while not scoopingResponse and not rospy.is_shutdown():
          busy.sleep() # 1 HZ
          scoopingResponse = self.TranslationalPTPMotionClient(startingPoint, startingPoint)

        while not str(self.scooping_status_.data) == 'DONE' and not str(self.scooping_status_.data)=='FAIL' and not rospy.is_shutdown():
          busy.sleep()

        if str(self.scooping_status_.data) == 'DONE':
          self._feedback.finished_scoops[scoop] = 1
        elif str(self.scooping_status_.data) == 'FAIL':
          self._feedback.finished_scoops[scoop] = 0

        rospy.loginfo(self.scooping_status_.data)
        rospy.loginfo(self._feedback.finished_scoops)

    print('last scoop is ', self._feedback.finished_scoops[len(self._feedback.finished_scoops)-1])

    # Scooped
    if self._feedback.finished_scoops[len(self._feedback.finished_scoops)-1]==1 and self.scooping_status_.data == 'DONE':
      success = True
    elif self.scooping_status_.data == 'FAIL':
      success = False

    # Send result
    self._result.success = success
    self._result.error_message = 'Error in scooping' if not success else "success"
    self.server_.set_succeeded(self._result)
    self._feedback.finished_scoops = []

    wentHome = self.GoHome()
    # while not str(self.scooping_status_.data) == 'IDLE':
    #   r.sleep()

    while not wentHome and not rospy.is_shutdown():
      busy.sleep() # 1 HZ
      wentHome = self.GoHome(startingPoint, startingPoint)


    if wentHome:
      rospy.loginfo('Going home')
    else:
      rospy.logwarn('Could not go home, the traffic is terrible')

    while not str(self.scooping_status_.data) == 'DONE' and not str(self.scooping_status_.data)=='FAIL' and not rospy.is_shutdown():
      r.sleep()

    self.__init__()

  # This function is called every _feedback_delay seconds
  def SendFeedbackLuigi(self, sc):
    if self.we_have_client_ and len(self._feedback.finished_scoops)>0:
      self._feedback.status_message = self.scooping_human_status_

      self._feedback.status_message = "more time"
      if self._feedback.finished_scoops[len(self._feedback.finished_scoops)-1] ==1:
        self._feedback.status_message = "Done"

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

  rospy.Timer(rospy.Duration(server._feedback_delay), server.SendFeedbackLuigi)
  # s.enter(server._feedback_delay, 1, server.SendFeedbackLuigi, (s,))
  s.run()
  print("Action server ready")
  rospy.spin()
