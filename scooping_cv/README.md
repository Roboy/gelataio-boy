# scooping_cv

## Ice Cream Detection

Given the desired flavour received from Luigi team, extract the corresponding point cloud and find best scooping point.

## Vision Module launch
To run the vision module, pull the image from docker hub
```bash
> docker pull hbdo/roboy_cv:v5
```

Execute this command to run the image with webcam and ToF camera. You may need to unload and reload uvcvideo kernel if the container can't detect the webcam.
```bash
#sudo modprobe -r uvcvideo
#sudo modprobe uvcvideo nodrop=1 timeout=5000 quirks=0x80

> docker run --privileged --network host -it hbdo/roboy_cv:v5 bash
```

Inside the container, you need to start ToF camera driver with this command:
```bash
root@container> roslaunch pico_flexx_driver pico_flexx_driver.launch
```

In another terminal, you can start the ice cream service with:
```bash
> docker exec -it <container_id> bash
root@container:/cv> python ice_cream_service.py
```

To make requests to the service, use this command from any terminal connected to the same ROS network:
```bash
> rosservice call /iceCreamMeshService "<flavor>"
```
You can test this by replacing `<flavor>` with `chocolate`, `flakes`, or `strawberry`

#### ROS topics and services
Communication protocol: ```roboy_cognition_msgs::DetectIceCream```

Service name: ```iceCreamService```

RVIZ visualization topics: ```/ice_cream_pc``` ```/ice_cream_scoop_point```

#### Usage
Starting vision as a service:
```rosrun scooping_cv ice_cream_service.py```

#### Example:
```$ rosrun scooping_cv ice_cream_service.py```


```
$ rosservice call /iceCreamService 'vanilla'
start_scooping: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: "torso"
  point: 
    x: -0.179830705523
    y: -0.594756553173
    z: 0.110840021819
end_scooping: 
  header: 
    seq: 0
    stamp: 
      secs: 0
      nsecs:         0
    frame_id: ''
  point: 
    x: 0.0
    y: 0.0
    z: 0.0
error_message: ''
```

## Tracking
Bayesian Object Tracking - follow the guide here: https://github.com/bayesian-object-tracking/getting_started

The .obj files are located in scooping_cv/meshes/tracking, which can be placed in dbot_getting_started/object_meshes to start testing. (Probably needs to be restructured later)

#### Usage
`roslaunch dbot_ros particle_tracker.launch` 

For best results, the parameters of the ZED camera needs to be tuned, an example config file can be found in the config folder. Copy the content of this file into the zed_wrapper/params package.

Starting tracker as service:        
`roslaunch dbot_ros object_tracker_service.launch`

calling the service:       
`rosrun scooping_cv dbot_tracking_client.py scooper` 

--> The tracking estimate is published under the topic 
`/object_tracker_service/object_state`

**NOTE**: WORK IN PROGRESS: For the moment the service doesn't provide any tracking results, it is merely a working ROS communication node
