//
// Created by arne on 05.09.19.
//

#include <std_msgs/String.h>
#include "gelataio_boy_control/emotions_interface.h"

emotions_interface::emotions_interface(ros::NodeHandle *nh) {
    emotion_pub = nh->advertise<std_msgs::String>("/roboy/cognition/face/show_emotion", 1);
}

void emotions_interface::show_emotion(std::string emotion) {
    std_msgs::String data;
    data.data = emotion;
    emotion_pub.publish(data);
}
