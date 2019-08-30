//
// Created by arne on 13.07.19.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <gelataio_boy_control/plan_executor.h>
#include <std_msgs/Float32.h>

CardsflowPlanExecutor::CardsflowPlanExecutor(ros::NodeHandle *nh) {
    joint_target_pub = nh->advertise<sensor_msgs::JointState>("/joint_targets", 1);
    ignored_joints.emplace_back("scooper_dummy_joint0");
    ignored_joints.emplace_back("scooper_dummy_joint1");
    ignored_joints.emplace_back("scooper_dummy_joint2");
}

bool CardsflowPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    ROS_INFO("Moving robot using CARDSflow");


    ros::Duration lastTime(0);

    for (const auto &state : plan.trajectory_.joint_trajectory.points) {
        ros::Duration dt = state.time_from_start - lastTime;
        dt.sleep();
        lastTime = state.time_from_start;

        sensor_msgs::JointState targets;
        for (int i = 0; i<plan.trajectory_.joint_trajectory.joint_names.size(); i++) {
            std::string name = plan.trajectory_.joint_trajectory.joint_names[i];
            if (std::find(ignored_joints.begin(), ignored_joints.end(), name) != ignored_joints.end()) continue;

            targets.name.push_back(name);
            targets.position.push_back(state.positions[i]);
            targets.velocity.push_back(state.velocities[i]);
        }

        joint_target_pub.publish(targets);
    }

    return true;
}
