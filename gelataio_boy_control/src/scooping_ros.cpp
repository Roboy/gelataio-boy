//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping_ros.h"

#include <thread>
#include <std_msgs/String.h>

using namespace geometry_msgs;
using namespace roboy_control_msgs;
using namespace std;


ScoopingROS::ScoopingROS(ros::NodeHandle *handle) : nh(handle), app(handle, false), busy(false), executor(nullptr), done(2) {}

void ScoopingROS::run() {
    scooping_srv = nh->advertiseService("scooping_planning/scoop", &ScoopingROS::scooping_cb, this);
    go_home_srv = nh->advertiseService("scooping_planning/go_home", &ScoopingROS::go_home_cb, this);
    init_pose_srv = nh->advertiseService("scooping_planning/init_pose", &ScoopingROS::init_pose_cb, this);
    status_pub = nh->advertise<std_msgs::String>("scooping_planning/status", 5);

    ROS_INFO("Scooping planning ready and waiting for service requests...");
    ros::Rate loop_rate(10.0);

    std_msgs::String status_msg;

    while (ros::ok()) {
        ros::spinOnce();
        status_msg.data = app.get_status();
        if (this->done == 1){
            status_msg.data = "DONE";
        } else if (this->done == 0) {
            status_msg.data = "FAIL";
        }

        status_pub.publish(status_msg);
        loop_rate.sleep();
    }
}

bool ScoopingROS::scooping_cb(TranslationalPTPMotion::Request &req, TranslationalPTPMotion::Response &resp) {
    stringstream ss;
    ss << "Got scooping request: " << req << std::endl;
    ROS_INFO_STREAM(ss.str());

    this->done=2;
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
                [this](int success) {this->busy = false; this->done = success;});
    }

    return true;
}

bool ScoopingROS::go_home_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    ROS_INFO("Go Home roboy, you are drunk ");

    this->done=2;
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
        executor = new std::thread(&ScoopingMain::go_home, &app, [this](bool success) {this->busy = false; 
            this->done=success;});
    }

    return true;
}

bool ScoopingROS::init_pose_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp) {
    ROS_INFO("Moving to start pose");

    resp.message = "Moving to init pose";

    if (this->busy) {
        resp.success = false;
        resp.message = "Can't run now, we are running something else on Roboy";
        ROS_ERROR_STREAM(resp.message);
    } else {
        if (this->executor != nullptr) {
            this->executor->join();
            delete this->executor;
            this->executor = nullptr;
        }
        resp.success = true;
        this->busy = true;
        executor = new std::thread(&ScoopingMain::init_pose, &app, [this](bool success) {this->busy = false;});
    }
}
