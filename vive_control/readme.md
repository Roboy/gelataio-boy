 
# Vive control
This is a ROS package to manually control a Roboy's arm with a VIVE controller to assist in situations, where path planning or regular control are inaccessible. 
 
It assumes, that you have a working kindyn with correct robot's model and working IK. (Installation: https://cardsflow.readthedocs.io/Usage/0_installation.html)
 
## Disclaimer
It was tested before the Roboy SS19 finals on `roboy_icecream`, but was slightly refactored afterwards without the access to some the equipment. If any issues arise - feel free to submit a new issue.
 
Also, it doesn't use the controller orientation, only position, because of the stability issues. Instead, you can manually move the wrist using the controller's trackpad.
 
## Other dependencies
 
**Steam**:
 
https://store.steampowered.com/about/ 
 
plus SteamVR 
 
https://store.steampowered.com/app/250820/SteamVR/
 
If you are using a laptop, make sure that it is VR-ready. 
 
Use this workaround if you don't have a SteamVR HMD: http://help.triadsemi.com/en/articles/836917-steamvr-tracking-without-an-hmd
 
Attention! This workaround may still not work on your hardware. In these situations, your only option is to attach a working headset.
 
**OpenVR**:
 
```
pip install openvr
```
 
 
**Triad OpenVR**:
 
```
git clone https://github.com/TriadSemi/triad_openvr
 
export PYTHONPATH=$PYTHONPATH:PATH_TO_YOUR_FOLDER/triad_openvr/
 
echo "export PYTHONPATH=$PYTHONPATH:PATH_TO_YOUR_FOLDER/triad_openvr/" >> ~/.bashrc
```
 
Don't forget to edit the path to your folder. 
 
And python packages that you probably already have, but still:
 
```
pip install numpy scipy pyyaml
```
 
## Configuration
 
Look at readme in `vive_control/scripts`
 
It is already configured for `roboy_icecream`, and it *should* be possible to easily configure it to work with other robots. 
 
## Usage
 
Start Steam VR, connect your controller to it, assure that is tracked by both lighthouses, and assume the initial position (your arm is bent in the elbow 90 degrees and controller is looking straight) and launch:
 
```
roslaunch vive_control assume_direct_control.launch
```
 
Press trigger if you want to reset the current robot's position to the initial one.
 
Use trackpad to manually rotate the wrist.
 
Use Rviz for visualization if necessary


### VR interface

Also, an interface for the VR team is implemented according to their specifications. It republisher poses of robot's right arm links to work with their advanced operator mode.

```
roslaunch vive_control republisher.launch
```

