cmake_minimum_required(VERSION 2.8.3)
project(dummy_publishers)

find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  geometry_msgs
  roscpp
  rospy
  rviz
  sensor_msgs
  std_msgs
  tf
  urdf
  xacro
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy rviz sensor_msgs std_msgs urdf xacro
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

