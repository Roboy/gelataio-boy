cmake_minimum_required(VERSION 2.8.3)
project( gelataio_msgs )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED cmake_modules COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        actionlib_msgs
        std_srvs
        message_generation
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)

#add_message_files(
#        DIRECTORY
#        msg
#        FILES
#)

add_service_files(
        DIRECTORY
        srv
        FILES
        PerformScoop.srv
)


generate_messages(
        DEPENDENCIES std_msgs std_srvs actionlib_msgs geometry_msgs
)

catkin_package( CATKIN_DEPENDS roscpp message_runtime actionlib_msgs
        INCLUDE_DIRS)
