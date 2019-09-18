 
# Configuration
 
Generally, you can use different configuration presets in different yaml files in this directory. Change `args` in `vive_tracking/launch/assume_direct_control.launch` for the name of the config file you want to use. 
 
`vive_control.yaml` is the default config file for the `roboy_icecream` robot
 
## Parameters
 
 
`controller_name` - a controller name, assigned by OpenVR. It assigns it in the order of connection to the Steam VR, so your first connected controller should be `controller_1`, your second one `controller_2` and so on.
 
`endeffector_name` - the name for the end effector with regard to which IK is calculated.
 
`inverse_kinematics_service` - the name of the service for the inverse kinematics calculation. Should have a type of `roboy_middleware_msgs.srv.InverseKinematics`
 
`robot_state_topic` - the topic of the current robot state, which has a list of all the poses of robot's links. Should have a type of `geometry_msgs.msg.PoseStamped`
 
`target_joint_angle_topic` - the topic of target joint angles for the robot's joints. Should have a type of `sensor_msgs.msg.JointState`
 
`wrist_axis_name` - the name of the robot's wrist joint. Necessary for the wrist control.
 
`joint_state_topic_name` - the topic of the current robot's joint angles. Should have a type of `sensor_msgs.msg.JointState` 
 
`wrist_motor_topic` - the topic for the target wrist angle if you are using direct motor control. Should have a type of `roboy_middleware_msgs.msg.MotorCommand` 
 
`wrist_simulated` - set to True if you want to use CARDSflow for the wrist angle control and simulate it if you have launched kindyn in the simulation mode.
 
`wrist_direct_motor_control` - set to True if you want to directly control wrist motor bypassing the simulation. 
 
Both of them should be True if you have not properly configured the wrist and only one should be true otherwise.
 
`filtering_alpha` - to smooth out the controlling signal, we are using exponential filtering. This is the alpha coefficient for the previous term for the said filter.
 
 
`wrist_max_angle` - maximum possible angle for the wrist in degrees. 
 
`wrist_max_angle` - minimum possible angle for the wrist in degrees. 
 
`sensitivity` - sensitivity with which operator movements are translated into the robot's movements. 

