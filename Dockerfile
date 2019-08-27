FROM missxa/melodic-dashing-roboy

RUN cd ~ && mkdir scooping_ws && cd scooping_ws && mkdir src && cd src && mkdir gelataio-boy
WORKDIR ~/scooping_ws/src
COPY . ./gelataio-boy

##CARDSflow dependencies
RUN apt update
RUN apt install --yes --force-yes libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings ros-$ROS_DISTRO-moveit-msgs doxygen swig mscgen ros-$ROS_DISTRO-grid-map ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-aruco-detect ros-$ROS_DISTRO-effort-controllers libxml++2.6-dev ros-$ROS_DISTRO-robot-localization libalglib-dev ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-rqt-gui ros-$ROS_DISTRO-rqt-gui-cpp gazebo9 libgazebo9-dev

RUN cd ~ && git clone https://github.com/nlohmann/json && cd json && mkdir build && cd build && cmake .. && make -j8 && make install

RUN git clone --recursive https://github.com/CARDSflow/CARDSflow
RUN cd CARDSflow/kindyn && git checkout master
RUN cd CARDSflow/roboy_communication && git checkout master
RUN cd CARDSflow/robots && git checkout hackathon
RUN cd CARDSflow/iDynTree && mkdir build && cd build && cmake .. && make -j9 && make install
RUN cd CARDSflow/qpOASES && mkdir build && cd build && cmake .. && make -j9 install

RUN apt install -y ros-melodic-moveit
RUN cd .. && catkin_make
