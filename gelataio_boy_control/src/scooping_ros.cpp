//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"

using namespace gelataio_msgs;
using namespace std;


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, true) {
    ros::ServiceServer scooping_srv = nh->advertiseService("scoop", &ScoopingROS::scooping_cb, this);
}

void ScoopingROS::run() {
    ros::spin();
}

bool ScoopingROS::scooping_cb(PerformScoop::Request &req, PerformScoop::Request &resp) {
    stringstream ss;
    ss << "Got scooping request: " << req << std::endl;
    ROS_INFO_STREAM(ss.str());
}

