//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_ROS_H
#define SRC_SCOOPING_ROS_H


#include <string>
#include <ros/node_handle.h>
#include "scooping.h"
#include <gelataio_msgs/PerformScoop.h>

class ScoopingROS {
public:
    explicit ScoopingROS(ros::NodeHandle *handle);

    void run();

    bool scooping_cb(gelataio_msgs::PerformScoop::Request &req, gelataio_msgs::PerformScoop::Request &resp);

private:
    ros::NodeHandle *nh;
    ScoopingMain app;
};


#endif //SRC_SCOOPING_ROS_H
