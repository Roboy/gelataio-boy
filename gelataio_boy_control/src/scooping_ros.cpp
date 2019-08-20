//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"

#include <thread>

using namespace gelataio_msgs;
using namespace geometry_msgs;
using namespace std;


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, true) {

}

void ScoopingROS::run() {
    scooping_srv = nh->advertiseService("scooping_planning/scoop", &ScoopingROS::scooping_cb, this);

    ROS_INFO("Scooping planning ready and waiting for service requests...");
    ros::spin();
}

bool ScoopingROS::scooping_cb(PerformScoop::Request &req, PerformScoop::Response &resp) {
    stringstream ss;
    ss << "Got scooping request: " << req << std::endl;
    ROS_INFO_STREAM(ss.str());

    resp.success = true;

    todo = new std::thread(&ScoopingMain::scoop_ice, &app, req.start_point, req.end_point);

    /*
     * Very important TODO: block creating of further threads before the first one finished.
     * Also the delete of the thread pointer is missing right now.
     * This is super ugly and was only a hack to check if everything blocks because of the pending respond.
     * ...which was indeed the case.
     */

    return true;
}


