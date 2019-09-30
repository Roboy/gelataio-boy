# :ice_cream: [Gelataio](https://en.wiktionary.org/wiki/gelataio) mode for [Roboy](https://roboy.org/) :ice_cream:

This is a repository with all the packages used by the scooping team while developing scooping procedure for Roboy. Here you can find everything regarding scooping, planning, vision, etc.


| Vision Video | Final Video |
| ------------ | ----------- |
|<a href="http://www.youtube.com/watch?feature=player_embedded&v=DI6FIhWma3Y" target="_blank"><img src="http://img.youtube.com/vi/DI6FIhWma3Y/0.jpg" alt="Vision Video" width="240" height="180" border="10" /></a> | <a href="http://www.youtube.com/watch?feature=player_embedded&v=F1awb4STJ84&list=PL5VpohfE5RnEzUAK9cNsspMwvD0zZquJI&index=4" target="_blank"><img src="http://img.youtube.com/vi/F1awb4STJ84/0.jpg" alt="Final Video" width="240" height="180" border="10" /></a> |

## Slides
- [Midterm Slides](https://docs.google.com/presentation/d/1NewMwSdnp7RwAgcC_RcqNRAtOPzeC24GHkpUbu42Q8k/edit?usp=sharing)
- [Finals Slides](https://docs.google.com/presentation/d/1lT7aWF8S_64XrxOK5uWU9qckEqH8yqs6vt6CNHx1nYM/edit#slide=id.g3ecef60b9d_0_7)
- [Sprint Sync 1](https://docs.google.com/presentation/d/12Hat28XKuapki89IOibCmz_zspT1Y5xjBFHNrMO4YSE/edit#slide=id.g3ec4627452_0_202)
- [Sprint Sync 2](https://docs.google.com/presentation/d/1Jai6Dpnfc-tcUIdtP4Eqru7uiEwxb5YKbDpsJK-65wQ/edit#slide=id.g3ecef60b9d_0_7)

## Package list

Our project was organized in semi-independent packages, most of which can be configured to perform other tasks:

- **coordinator** - a.k.a. "workflow module" responsible to the coordination of all other submodules, interfacing with Luigi (human interaction package), services and message delivery
- **gelataio_boy_control** - main control package for arm control and path execution
- **moveit_roboy_icecream** - configuration files for moveit and path planning

These packages should be used simultaneously using the launch procedure from below. We also have two supplementary packages with their own user manuals in respecting folders:
- **scooping_cv** - computer vision package for icecream recognition and surface detection
- **vive_control** - package for direct arm control using a VR controller

## Architecture 

![Arch Diagram](doc/arch.png)

# Installation using Docker


Our main packages are supposed to be used inside a docker container. You can pull it from the hub using the following command:
```
docker pull stlukyanenko/gelataio-control:latest
```

And run the pulled image using:
```
docker run --privileged --network host -it stlukyanenko/gelataio-control:latest
```

# Manual installation

Generally, the docker image should suffice, but if you want to run it without it, you need to:

[Install CARDSflow](https://cardsflow.readthedocs.io/Usage/0_installation.html)

Checkout the branches of it's submodules to the branches used by the control package:

```
kindyn => friday-branch
roboy_communication => master
robots => friday-branch
```

and install MoveIt:

```
sudo apt install ros-$ROS_DISTRO-moveit ros-$ROS_DISTRO-moveit-visual-tools  ros-$ROS_DISTRO-gazebo-plugins
```


# Launch Files for Startup of all processes

We provide a set of launch files launching a couple of ROS nodes dependent on which setup you have. We distinguish real and sim environments. Real means for have roboy and the FPGA's running and have the hardware in the loop during execution. For sim environments there is no Roboy hardware and you use rviz to show the world state, e.g. you are in a simulated environment.

| Real / Simulated Environment | Luigi Available? | Launch File |
| ---------------------------- | ---------------- | ----------- |
| Real | Yes | [start_real.launch](coordinator/launch/start_real.launch) |
| Real | No (fake him) | [start_real_test.launch](coordinator/launch/start_real_test.launch)|
| Sim | Yes | [start_sim.launch](coordinator/launch/start_sim.launch)|
| Sim | No (fake hime) | [start_sim_test.launch](coordinator/launch/start_sim_test.launch)|

# Code Organization

Clone the repository into the `src` folder of your catkin workspace. Like this
```
cakin_ws/
	build/
		...
	devel/
		...
	src/
		CARDSflow/
			...
		gelatatio-boy/
			<subpackages of our project>
		<further packages if required>
```

Catkin build the packages recursively.



