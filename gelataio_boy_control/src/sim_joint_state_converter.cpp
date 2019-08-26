//
// Created by arne on 26.08.19.
//

#include <iostream>
#include <ros/ros.h>
#include <roboy_simulation_msgs/JointState.h>
#include <sensor_msgs/JointState.h>

using namespace std;

static ros::Publisher pub;
static vector<string> joint_names;

void jointStateCallback(const roboy_simulation_msgs::JointState &state) {
    sensor_msgs::JointState s;
    for (int i = 0; i < state.names.size(); i++) {
        s.name.push_back(state.names[i]);
        s.position.push_back(state.q[i]);
        s.velocity.push_back(state.qd[i]);
    }
    for (int i = 0; i < 3; i++) {
        stringstream ss; ss << "scooper_dummy_joint" << i;
        s.name.push_back(ss.str());
        s.position.push_back(0.0);
        s.velocity.push_back(0.0);
    }
    s.header.stamp = ros::Time::now();
    pub.publish(s);
    ROS_INFO_THROTTLE(10.0, "I'm alive.");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sim_joint_state_converter");
    ros::NodeHandle nh;

    if (ros::param::has("joint_names")) {
        auto sub = nh.subscribe("/joint_state", 10, jointStateCallback);
        pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

        ros::param::get("joint_names", joint_names);
        stringstream ss; ss << "Joint names are: [";
        int i = 0;
        for (const auto &s : joint_names) ss << "\t" << i++ << ": " << s << endl;
        ROS_INFO_STREAM(ss.str());

        ROS_INFO("Converted started successfully.");
        ros::spin();
    } else {
        ROS_ERROR("No parameter joint_names.");
        return -1;
    }
}