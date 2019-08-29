//
// Created by arne on 26.08.19.
//


#include <ros/ros.h>
#include <gelataio_boy_control/JointInfoCollector.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "joint_info_collector");
    ros::NodeHandle handle;

    JointInfoCollector app(&handle);

    ros::spin();
}