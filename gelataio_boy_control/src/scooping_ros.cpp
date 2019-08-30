//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"

#include <thread>
#include <std_msgs/String.h>

using namespace geometry_msgs;
using namespace roboy_control_msgs;
using namespace std;


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, false), busy(false), executor(nullptr) {}

void ScoopingROS::run() {
    scooping_srv = nh->advertiseService("scooping_planning/scoop", &ScoopingROS::scooping_cb, this);
    go_home_srv = nh->advertiseService("scooping_planning/go_home", &ScoopingROS::go_home_cb, this);
    status_pub = nh->advertise<std_msgs::String>("scooping_planning/status", 5);

    ROS_INFO("Scooping planning ready and waiting for service requests...");
    ros::Rate loop_rate(10.0);

    std_msgs::String status_msg;

    while (ros::ok()) {
        ros::spinOnce();
        status_msg.data = app.get_status();
        status_pub.publish(status_msg);
        loop_rate.sleep();
    }
}

bool ScoopingROS::scooping_cb(TranslationalPTPMotion::Request &req, TranslationalPTPMotion::Response &resp) {
    stringstream ss;
    ss << "Got scooping request: " << req << std::endl;
    ROS_INFO_STREAM(ss.str());

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
        executor = new std::thread(&ScoopingMain::scoop_ice, &app, req.start_point, req.end_point,
                [this](bool success) {this->busy = false;});
    }

    return true;
}

bool ScoopingROS::go_home_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    ROS_INFO("Go Home roboy, you are drunk ");

    resp.message = "Going home";

    if (this->busy) {
        resp.success = false;
        ROS_ERROR("Can't go home right now, a scooping is still running.");
    } else {
        if (this->executor != nullptr) {
            this->executor->join();
            delete this->executor;
            this->executor = nullptr;
        }
        resp.success = true;
        this->busy = true;
        executor = new std::thread(&ScoopingMain::go_home, &app, [this](bool success) {this->busy = false;});
    }

    return true;
}
