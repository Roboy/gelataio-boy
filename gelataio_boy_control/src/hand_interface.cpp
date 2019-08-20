//
// Created by arne on 16.08.19.
//

#include <sstream>
#include "gelataio_boy_control/hand_interface.h"
#include <ros/ros.h>

bool DummyHand::grasp() {
    std::stringstream ss;
    ss << "Closing hand " << hand_name << ".";
    ROS_INFO_STREAM(ss.str());
    return true;
}

bool DummyHand::release() {
    std::stringstream ss;
    ss << "Opening hand " << hand_name << ".";
    ROS_INFO_STREAM(ss.str());
    return true;
}
