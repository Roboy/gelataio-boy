//
// Created by arne on 26.08.19.
//

#include <iostream>
#include <ros/ros.h>
#include <roboy_simulation_msgs/JointState.h>
#include <sensor_msgs/JointState.h>

using namespace std;

static ros::Publisher pub;
const static vector<int> joint_ids = {0, 1, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
static vector<string> joint_names;

void jointStateCallback(const roboy_simulation_msgs::JointState &state) {
    sensor_msgs::JointState s;
    for (const int index : joint_ids) {
        s.name.push_back(joint_names[index]);
        s.position.push_back(state.q[index]);
        s.velocity.push_back(state.qd[index]);
    }
    pub.publish(s);
    ROS_INFO_THROTTLE(10.0, "I'm alive.");
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sim_joint_state_converter");
    ros::NodeHandle nh;

    if (ros::param::has("joint_names")) {
        auto sub = nh.subscribe("/joint_state", 10, jointStateCallback);
        pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
        ROS_INFO("Converted started successfully.");

        ros::param::get("joint_names", joint_names);
        stringstream ss; ss << "Joint names are: [";
        int i = 0;
        for (const auto &s : joint_names) ss << "\t" << i++ << ": " << s << endl;
        ROS_INFO_STREAM(ss.str());

        ros::spin();
    } else {
        ROS_ERROR("No parameter joint_names.");
        return -1;
    }
}