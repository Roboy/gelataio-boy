#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import Header
from dbot_ros_msgs.srv import RunMultiObjectTracker
from dbot_ros_msgs.msg import ObjectState
from dbot_ros_msgs.msg import ObjectOri
from dbot_ros_msgs.msg import MultiObjectState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers


ar_marker = True

def track_object(object_name):

  print("object names length: {}".format(len(object_name)))

  print("Waiting for service...")
  rospy.wait_for_service("/multi_object_tracker_service")

  if ar_marker:
    print("Waiting for init pose...")

    marker_msg_array = rospy.wait_for_message("ar_pose_marker",AlvarMarkers)
    print("Waiting for init pose done...")

    while(len(marker_msg_array.markers)==0):
      marker_msg_array = rospy.wait_for_message("ar_pose_marker",AlvarMarkers)

    marker_msg = marker_msg_array.markers[0]

  try:
    run_object_tracker = rospy.ServiceProxy('/multi_object_tracker_service', RunMultiObjectTracker)

    # set initial pose
    pose=[]

    if ar_marker:
      pose.append(marker_msg.pose.pose)
    else:
      pose.append(Pose(position=Point(0, 0.1, 0.7),
                  orientation=Quaternion(0, 0, 0, 0)))
      pose.append(Pose(position=Point(-0.1, 0.1, 0.7),
                  orientation=Quaternion(0, 0, 0, 0)))
      

    # set Object resource identifier to find the model
    ori =[]
    
    pose_stamped=[]

    # construct the ObjectState message for the service call
    object_state = MultiObjectState()

    for i in range(len(object_name)):
      print(i)
      print(object_name[i])

      ori.append(ObjectOri(package = "scooping_cv",
                    directory="meshes/tracking",
                    name=object_name[i] + ".obj"))

      # ori.append(ObjectOri(package = "scooping_cv",
      #               directory="meshes/tracking",
      #               name=object_name[2] + ".obj"))


      pose_stamped.append(PoseStamped(
                   pose=pose[i],
                   header=Header(seq=0, stamp=rospy.Time(0), frame_id=str(i) )))
      # pose_stamped.append(PoseStamped(
      #              pose=pose[1],
      #              header=Header(seq=0, stamp=rospy.Time(0), frame_id='')))

      

      object_state.name.append(object_name[i])
      # object_state.name.append(object_name[2])
      
      object_state.ori.append(ori[i])
      # object_state.ori.append(ori[1])
    
      object_state.pose.append(pose_stamped[i])
      # object_state.pose.append(pose_stamped[1])



    print("Calling tracker service to track the %s object" % object_name)
    run_object_tracker(object_state)


  except rospy.ServiceException, e:
      print("Calling object tracker service failed: %s" % e)

if __name__ == "__main__":
  rospy.init_node("tracking_client")

  track_object(sys.argv[1:])

  # rospy.spin()
  #track_object(sys.argv) #for more arguments