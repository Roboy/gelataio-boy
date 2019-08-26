//
// Created by arne on 26.08.19.
//

#include <sensor_msgs/JointState.h>
#include "gelataio_boy_control/JointInfoCollector.h"


JointInfoCollector::JointInfoCollector(ros::NodeHandle *nh) {
    joint_states_pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);
    steering_angle_sub = nh->subscribe("/roboy/middleware/RickshawAngle", 10, &JointInfoCollector::steering_angle_cb, this);
}

void JointInfoCollector::steering_angle_cb(const std_msgs::Float32 &data) {
    sensor_msgs::JointState s;

    s.header.stamp = ros::Time::now();
    s.name.push_back("steering_angle");
    s.position.push_back(data.data);
    s.velocity.push_back(0.0);

    joint_states_pub.publish(s);
}
