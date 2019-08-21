//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"

#include <thread>

using namespace gelataio_msgs;
using namespace geometry_msgs;
using namespace std;


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, true), busy(false), executor(nullptr) {

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

    resp.success = !this->busy;

    if (this->busy) {
        resp.success = false;
        ROS_ERROR("Can't start another scooping action while still running.");
    } else {
        if (this->executor != nullptr) {
            this->executor->join();
            delete this->executor;
            this->executor = nullptr;
        }
        resp.success = true;
        this->busy = true;
        auto do_this_when_finished = [this](bool success) {this->busy = false;};
        executor = new std::thread(&ScoopingMain::scoop_ice, &app, req.start_point, req.end_point, do_this_when_finished);
    }

    return true;
}


