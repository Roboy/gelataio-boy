//
// Created by arne on 05.09.19.
//

#ifndef SRC_EMOTIONS_INTERFACE_H
#define SRC_EMOTIONS_INTERFACE_H


#include <ros/ros.h>

class emotions_interface {
public:
    explicit emotions_interface(ros::NodeHandle *nh);

    void show_emotion(std::string emotion);

private:
    ros::Publisher emotion_pub;

};


#endif //SRC_EMOTIONS_INTERFACE_H
