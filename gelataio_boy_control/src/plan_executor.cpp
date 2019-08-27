//
// Created by arne on 13.07.19.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <gelataio_boy_control/plan_executor.h>
#include <std_msgs/Float32.h>

CardsflowPlanExecutor::CardsflowPlanExecutor(ros::NodeHandle *nh) {
    joint_target_pub = nh->advertise<sensor_msgs::JointState>("/joint_targets", 1);
}

bool CardsflowPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    ROS_INFO("Moving robot using CARDSflow");

    sensor_msgs::JointState targets;
    targets.name = plan.trajectory_.joint_trajectory.joint_names;

    ros::Duration lastTime(0);

    for (const auto &state : plan.trajectory_.joint_trajectory.points) {
        ros::Duration dt = state.time_from_start - lastTime;
        dt.sleep();
        lastTime = state.time_from_start;

        targets.position = state.positions;
        targets.velocity = state.velocities;
        joint_target_pub.publish(targets);
    }

    return true;
}
