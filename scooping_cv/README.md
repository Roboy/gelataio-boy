# scooping_cv
SS19 Computer Vision - See the Icecream

<<<<<<< HEAD
## Tracking dependencies

**Bayesian Object Tracking**     
follow the guide here: https://github.com/bayesian-object-tracking/getting_started

The .obj files are located in scooping_cv/meshes/tracking

To get real-time results, a GPU should be used.


**Eigen 3.2**      
In order to install Eigen 3.2.10, download http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2.

run following codes:

$ cd catkin_ws/src

download using

$ wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2

$ tar -xf 3.2.10.tar.bz2    
$ cd eigen-eigen-b9cd8366d4e8    
$ mkdir build   
$ cd build    
$ cmake ..   
$ sudo make install

## Camera choice

THe problem with the ZED camera is that it cannot track homogeneous surfaces, e.g. the cup. The cup itself will be detected as flat surface, which corrupts the bayesian object tracking.

Using the Intel Realsense R200 (which has an infrared depth sensor), the cup is recognized without problems. However, the scooping tool has a special surface that does not go well with infrared - it is not detectable in the depth cloud. A temporary solution is to wrap the handle in paper, however tracking is still not accurate enough. This needs to be solved. An alternative would be using AR Tags.

Note on the realsense r200:
https://github.com/IntelRealSense/realsense-ros/issues/386    
Here is how to get the camera work with the old librealsense package version -- 
if using ROS kinetic,   
`sudo apt-get install ros-kinetic-librealsense`    
and    
`sudo apt-get install ros-kinetic-realsense_camera`     
should work

## Transform coordinate to robot reference frame
THe magic happens in the src/tracking/multi_obj_state_to_pose.py file, where the coordinates of the tracked object will be converted to a common world frame. As of 14th July 2019 21:14, we still need to change the name of the camera frame in line 28 to the proper camera frame name. Right now it is set to "torso" because the camera frame does not exist in the current urdf file.


## Example Usage
If running it in a docker container, don't forget to mount the container using priveleged mode:
`docker run -it --privileged -v /dev/bus/usb:/dev/bus/usb <container_name>`

Start camera:  
`roslaunch realsense_camera r200_nodelet_modify_params.launch` 
 
Or to run it using D435 and realsense 2, use 
`roslaunch realsense2_camera rs_camera.launch`

Start service:    
`roslaunch scooping_cv multi_object_tracker_service.launch`

Start visualization stuff:    
`roslaunch scooping_cv rviz_tracking.launch`    


Start AR Track Alvar for initial pose:    
`roslaunch scooping_cv rs_ar_track_alvar.launch` 

Call tracking service (track cup):   
`rosrun scooping_cv multi_tracking_client.py cup`

--> The tracking estimate is published under the topic 
`/object_tracker_service/object_state` 
and the object pose under:  
`/object_pose`

<!-- <node name="foo_throttler" type="throttle" pkg="topic_tools" args="messages /camera/depth/image_raw 10.0" /> -->
=======
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
>>>>>>> master
