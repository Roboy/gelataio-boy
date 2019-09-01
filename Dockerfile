FROM missxa/melodic-dashing-roboy

SHELL ["/bin/bash", "-c"] 

RUN cd /root && mkdir scooping_ws && cd scooping_ws && mkdir src && cd src && mkdir gelataio-boy

WORKDIR /root/scooping_ws/src

#CARDSflow dependencies
RUN apt update
RUN apt install --yes --force-yes gdb libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings ros-$ROS_DISTRO-moveit-msgs doxygen swig mscgen ros-$ROS_DISTRO-grid-map ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-aruco-detect ros-$ROS_DISTRO-effort-controllers libxml++2.6-dev ros-$ROS_DISTRO-robot-localization libalglib-dev ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-rqt-gui ros-$ROS_DISTRO-rqt-gui-cpp gazebo9 libgazebo9-dev

RUN cd /root && git clone https://github.com/nlohmann/json && cd json && mkdir build && cd build && cmake .. && make -j8 && make install

RUN git clone --recursive https://github.com/CARDSflow/CARDSflow
RUN cd CARDSflow/kindyn && git checkout master
RUN cd CARDSflow/roboy_communication && git checkout master
RUN cd CARDSflow/robots && git checkout hackathon
RUN cd CARDSflow/iDynTree && mkdir build && cd build && cmake .. && make -j9 && make install
RUN cd CARDSflow/qpOASES && mkdir build && cd build && cmake .. && make -j9 install

RUN apt install --yes --force-yes ros-melodic-moveit
RUN apt install --yes --force-yes ros-$ROS_DISTRO-moveit-visual-tools ros-$ROS_DISTRO-gazebo-plugins

#Eigen for bayesian object tracking
RUN wget http://bitbucket.org/eigen/eigen/get/3.2.10.tar.bz2
RUN tar -xf 3.2.10.tar.bz2
RUN cd eigen-eigen-b9cd8366d4e8 && mkdir build && cd build && cmake .. && make install

#Bayesian object tracking
RUN git clone https://github.com/filtering-library/fl
RUN git clone https://github.com/bayesian-object-tracking/dbot
RUN cd dbot && git checkout xenial-xerus-dev

RUN git clone https://github.com/bayesian-object-tracking/dbot_ros
RUN cd dbot_ros && git checkout xenial-xerus-kinetic-dev

RUN git clone https://github.com/bayesian-object-tracking/dbot_ros_msgs
RUN cd dbot_ros_msgs && git checkout xenial-xerus-kinetic-dev

#Realsense camera libraries
RUN apt-get install --yes --force-yes --install-recommends ros-$ROS_DISTRO-ddynamic-reconfigure
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
RUN add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u
RUN apt-get --yes --force-yes install librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
RUN git clone https://github.com/IntelRealSense/realsense-ros.git


RUN source /opt/ros/melodic/setup.bash && cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release -DDBOT_BUILD_GPU=On
