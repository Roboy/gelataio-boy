//
// Created by arne on 26.08.19.
//

#ifndef SRC_JOINTINFOCOLLECTOR_H
#define SRC_JOINTINFOCOLLECTOR_H


#include <ros/node_handle.h>
#include <std_msgs/Float32.h>

class JointInfoCollector {
public:
    explicit JointInfoCollector(ros::NodeHandle *nh);

protected:
    void steering_angle_cb(const std_msgs::Float32 &data);

private:
    ros::Publisher joint_states_pub;

    ros::Subscriber steering_angle_sub;
};


#endif //SRC_JOINTINFOCOLLECTOR_H
