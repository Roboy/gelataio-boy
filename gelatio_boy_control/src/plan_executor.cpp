//
// Created by arne on 13.07.19.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <gelataio_boy_control/plan_executor.h>
#include <std_msgs/Float32.h>

CardsflowPlanExecutor::CardsflowPlanExecutor(std::string group_name, ros::NodeHandle *nh) : group_name(group_name) {
    std::string axis0_prefix, axis1_prefix, axis2_prefix, elbow_prefix;
    axis0_prefix = "shoulder_" + group_name + "_axis0";
    axis1_prefix = "shoulder_" + group_name + "_axis1";
    axis2_prefix = "shoulder_" + group_name + "_axis2";
    elbow_prefix = "elbow_" + group_name;
    auto create_pub = [&](std::string prefix) {
        return std::make_pair(prefix,
                nh->advertise<std_msgs::Float32>("/" + prefix + "/" + prefix + "/target", 1));
    };
    publishers.insert(create_pub("shoulder_" + group_name + "_axis0"));
    publishers.insert(create_pub("shoulder_" + group_name + "_axis1"));
    publishers.insert(create_pub("shoulder_" + group_name + "_axis2"));
    publishers.insert(create_pub("elbow_" + group_name));
    publishers.insert(create_pub("wrist_" + group_name + "_axis0"));
    publishers.insert(create_pub("wrist_" + group_name + "_axis1"));
    publishers.insert(create_pub("wrist_" + group_name + "_axis2"));

}

bool CardsflowPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    std::cout << "Moving " << group_name << " arm using CARDSflow." << std::endl;

    for (int t = 0; t<plan.trajectory_.joint_trajectory.points.size(); t++) {
        if (t > 0) {
            ros::Duration dt = plan.trajectory_.joint_trajectory.points[t].time_from_start
                    - plan.trajectory_.joint_trajectory.points[t-1].time_from_start;
            dt.sleep();

        }
        for (int i = 0; i<plan.trajectory_.joint_trajectory.joint_names.size(); i++) {
            std::string joint_name = plan.trajectory_.joint_trajectory.joint_names[i];
            std_msgs::Float32 value;
            value.data = plan.trajectory_.joint_trajectory.points[t].positions[i];
            publishers.at(joint_name).publish(value); //TODO
        }
    }
    return true;
}
