cmake_minimum_required(VERSION 2.8.3)
project( gelataio_messages )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
        roscpp
        std_msgs
        std_srvs
        message_generation
        actionlib_msgs
        geometry_msgs
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

add_message_files(
        DIRECTORY
        msg
        FILES
)

add_service_files(
        DIRECTORY
        srv
        FILES
        PerformScoop.srv
)

add_action_files(
        DIRECTORY
        action
        FILES
)

generate_messages(
        DEPENDENCIES std_msgs std_srvs actionlib_msgs geometry_msgs
)

catkin_package( CATKIN_DEPENDS roscpp message_runtime message_generation actionlib_msgs
        INCLUDE_DIRS)
