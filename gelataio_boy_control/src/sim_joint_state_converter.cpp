//
// Created by arne on 26.08.19.
//

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

using namespace std;

static ros::NodeHandle *nh;
static ros::Publisher pub;
static vector<string> joint_names;
static bool do_angle_conversion;
static bool use_external_steering_angle;
static double steering_angle = 0;

void jointStateCallback(const sensor_msgs::JointState &state) {
    sensor_msgs::JointState s;

    for (int i = 0; i < state.name.size(); i++) {
        s.name.push_back(state.name[i]);
        s.velocity.push_back(state.velocity[i]);

        if (use_external_steering_angle && (state.name[i] == "bike_steering")) {
            s.position.push_back(steering_angle);
        } else if (do_angle_conversion && (state.name[i] == "wrist_right")) {
            s.position.push_back(state.position[i]);
        } else if ((state.name[i] == "elbow_right")) {
            double angle = state.position[i];
            if (angle > M_PI) angle -= 2*M_PI;
            s.position.push_back(angle);
        } else {
            s.position.push_back(state.position[i]);
        }
    }
    s.header.stamp = ros::Time::now();
    pub.publish(s);
    ROS_INFO_THROTTLE(10.0, "I'm alive.");
}

void rickshawStateCallback(const std_msgs::Float32 &data) {
    steering_angle = data.data;
    ROS_INFO_STREAM_THROTTLE(10.0, "Received steering angles: " << steering_angle);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "sim_joint_state_converter");
    nh = new ros::NodeHandle;

    nh->getParam("do_angle_conversion", do_angle_conversion);
    nh->getParam("use_external_steering_angle", use_external_steering_angle);

    if (ros::param::has("joint_names")) {
        auto sub = nh->subscribe("/cardsflow_joint_states", 10, jointStateCallback);
        ros::Subscriber sub2;
        if(use_external_steering_angle) {
          ROS_INFO("Subscribing rickshaw angle");
          sub2 = nh->subscribe("/roboy/middleware/RickshawAngle", 10, rickshawStateCallback);
        }

        pub = nh->advertise<sensor_msgs::JointState>("/joint_states", 10);

        ros::param::get("joint_names", joint_names);
        stringstream ss; ss << "Joint names are:";
        int i = 0;
        for (const auto &s : joint_names) ss << "\t" << i++ << ": " << s << endl;
        ROS_INFO_STREAM(ss.str());

        ROS_INFO("Converter started successfully.");
        ros::spin();
    } else {
        ROS_ERROR("No parameter joint_names.");
        return -1;
    }
}
