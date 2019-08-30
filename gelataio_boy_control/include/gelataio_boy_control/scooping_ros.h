//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_ROS_H
#define SRC_SCOOPING_ROS_H


#include <string>
#include <future>
#include <ros/node_handle.h>
#include "scooping.h"
#include <roboy_control_msgs/TranslationalPTPMotion.h>
#include <std_srvs/Empty.h>


class ScoopingROS {
public:
    explicit ScoopingROS(ros::NodeHandle *handle);

    void run();

    bool scooping_cb(roboy_control_msgs::TranslationalPTPMotion::Request &req, roboy_control_msgs::TranslationalPTPMotion::Response &resp);
    bool go_home_cb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);


private:
    ros::NodeHandle *nh;
    ScoopingMain app;

    ros::ServiceServer scooping_srv;
    ros::ServiceServer go_home_srv;
    ros::Publisher status_pub;

    bool busy;
    std::thread *executor;
};


#endif //SRC_SCOOPING_ROS_H
