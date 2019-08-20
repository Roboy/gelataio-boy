#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <gelataio_boy_control/scooping.h>

#include "gelataio_boy_control/hand_controller.hpp"

int main(int argc, char **argv) {

    ros::init(argc, argv, "gelataio_arm_controller");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ScoopingMain app(&node_handle, true);

    geometry_msgs::Point start, end;
    start.x = -0.27;
    start.y = -0.5;
    start.z = 0.6;
    end = start;

    app.scoop_ice(start, end);

}
