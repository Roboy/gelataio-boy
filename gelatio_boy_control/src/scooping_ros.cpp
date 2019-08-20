//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, true) {
    ros::ServiceServer scooping_srv = nh->advertiseService("scoop", &ScoopingROS::scooping_cb, this);
}

void ScoopingROS::run() {
    ros::spin();
}

bool ScoopingROS::scooping_cb(gelataio_msgs::PerformScoop::Request &req, gelataio_msgs::PerformScoop::Request &resp) {
    return false;
}

