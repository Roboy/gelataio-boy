# scooping_cv
SS19 Computer Vision - See the Icecream

## Tracking
Bayesian Object Tracking - follow the guide here: https://github.com/bayesian-object-tracking/getting_started

The .obj files are located in scooping_cv/meshes/tracking, which can be placed in dbot_getting_started/object_meshes to start testing. (Probably needs to be restructured later)

Usage:
`roslaunch dbot_ros particle_tracker.launch` 

For best results, the parameters of the ZED camera needs to be tuned, an example config file can be found in the config folder. Copy the content of this file into the zed_wrapper/params package.

Starting tracker as service:        
`roslaunch dbot_ros object_tracker_service.launch`

calling the service:       
`rosrun scooping_cv dbot_tracking_client.py scooper` 

--> The tracking estimate is published under the topic 
`/object_tracker_service/object_state`

**NOTE**: WORK IN PROGRESS: For the moment the service doesn't provide any tracking results, it is merely a working ROS communication node