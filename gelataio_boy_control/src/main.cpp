#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gelataio_boy_control/scooping.h>
#include <gelataio_boy_control/scooping_ros.h>

#include "gelataio_boy_control/hand_controller.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "scooping_planning");
    ros::NodeHandle node_handle;

    ScoopingROS scooping_ros_interface(&node_handle);
    scooping_ros_interface.run();

    geometry_msgs::Point start, end;
    start.x = -0.27;
    start.y = -0.5;
    start.z = 0.6;
    end = start;


}
