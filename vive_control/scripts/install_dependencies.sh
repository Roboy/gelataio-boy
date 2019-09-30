#!/bin/bash

apt update
apt install --yes --force-yes gdb libeigen3-dev libxml2-dev coinor-libipopt-dev qtbase5-dev qtdeclarative5-dev qtmultimedia5-dev qml-module-qtquick2 qml-module-qtquick-window2 qml-module-qtmultimedia qml-module-qtquick-dialogs qml-module-qtquick-controls qml-module-qt-labs-folderlistmodel qml-module-qt-labs-settings ros-$ROS_DISTRO-moveit-msgs doxygen swig mscgen ros-$ROS_DISTRO-grid-map ros-$ROS_DISTRO-controller-interface ros-$ROS_DISTRO-controller-manager ros-$ROS_DISTRO-aruco-detect ros-$ROS_DISTRO-effort-controllers libxml++2.6-dev ros-$ROS_DISTRO-robot-localization libalglib-dev ros-$ROS_DISTRO-tf ros-$ROS_DISTRO-interactive-markers ros-$ROS_DISTRO-tf-conversions ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-roslint ros-$ROS_DISTRO-rqt-gui ros-$ROS_DISTRO-rqt-gui-cpp gazebo9 libgazebo9-dev

PATH_HERE = dirname "$0"

cd ~
mkdir vive_control_dependencies
cd vive_control_dependencies
git clone https://github.com/nlohmann/json && cd json && mkdir build && cd build && cmake .. && make -j8 && make install
cd ..

pip install openvr

git clone https://github.com/TriadSemi/triad_openvr
export PYTHONPATH=$PYTHONPATH:~/vive_control_dependencies/triad_openvr/
echo "export PYTHONPATH=$PYTHONPATH:~/vive_control_dependencies/triad_openvr/" >> ~/.bashrc
pip install openvr
pip install numpy scipy pyyaml

cd $PATH_HERE
cd ../../..
git clone --recursive https://github.com/CARDSflow/CARDSflow
cd ./CARDSflow/kindyn && git checkout friday-branch
cd ..
cd roboy_communication && git checkout master
cd ..
cd roboy_communication && git checkout master && mkdir build && cd build && cmake .. && make -j9 && make install
cd ../..
cd robots && git checkout friday-branch
cd ..
cd iDynTree && mkdir build && cd build && cmake .. && make -j9 && make install
cd ../..
cd qpOASES && mkdir build && cd build && cmake .. && make -j9 install

