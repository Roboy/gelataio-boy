//
// Created by arne on 13.07.19.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <gelataio_boy_control/plan_executor.h>
#include <std_msgs/Float32.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#define rad2deg(rad) (rad/M_PI)*180.0

using namespace std;

CardsflowPlanExecutor::CardsflowPlanExecutor(ros::NodeHandle *nh) : nh(nh) {
    joint_target_pub = nh->advertise<sensor_msgs::JointState>("/joint_targets", 1);
    motor_command_pub = nh->advertise<roboy_middleware_msgs::MotorCommand>("/roboy/middleware/MotorCommand", 1);
    ignored_joints.emplace_back("scooper_dummy_joint0");
    ignored_joints.emplace_back("scooper_dummy_joint1");
    ignored_joints.emplace_back("scooper_dummy_joint2");

    fake_wrist = true;
}

bool CardsflowPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    ROS_INFO("Moving robot using CARDSflow");

    ros::Duration lastTime(0);

    auto &traj = plan.trajectory_.joint_trajectory;

    for (const auto &state : traj.points) {
        ros::Duration dt = state.time_from_start - lastTime;
        dt.sleep();
        lastTime = state.time_from_start;

        sensor_msgs::JointState targets;
        for (int i = 0; i<traj.joint_names.size(); i++) {
            std::string name = traj.joint_names[i];
            if (std::find(ignored_joints.begin(), ignored_joints.end(), name) != ignored_joints.end()) continue;

            targets.name.push_back(name);
            targets.position.push_back(state.positions[i]);
            targets.velocity.push_back(state.velocities[i]);
        }

        auto wrist_joint = std::find(traj.joint_names.begin(),
                traj.joint_names.end(), "wrist_right");

        if (wrist_joint != plan.trajectory_.joint_trajectory.joint_names.end()) {
            int wrist_index = std::distance(traj.joint_names.begin(), wrist_joint);

            roboy_middleware_msgs::MotorCommand cmd;
            cmd.id = 6;
            cmd.motors.push_back(2);
            cmd.set_points.push_back(rad2deg(state.positions[wrist_index]));
            if (fake_wrist) {
                nh->setParam("wrist_right_angle", cmd.set_points[0]);
            } else {
                ROS_WARN_THROTTLE(2.0, "Faking wrist.");
                motor_command_pub.publish(cmd);
            }
        } else {
            ROS_WARN("Wrist right not found in the plan.");
        }

        joint_target_pub.publish(targets);
    }

    return true;
}

bool CardsflowPlanExecutor::moveJointTo(std::string joint_name, double target) {
    map<string, double> m;
    m[joint_name] = target;
    return this->moveJointsTo(m);
}

bool CardsflowPlanExecutor::moveJointsTo(const std::map<std::string, double> &target) {
    sensor_msgs::JointState js;
    for (const auto &pair : target) {
        if (std::find(ignored_joints.begin(), ignored_joints.end(), pair.first) == ignored_joints.end()) {
            js.name.push_back(pair.first);
            js.position.push_back(pair.second);
            js.velocity.push_back(0.0);
            if (pair.first == "wrist_right") {
                roboy_middleware_msgs::MotorCommand cmd;
                cmd.id = 6;
                cmd.motors.push_back(2);
                cmd.set_points.push_back(rad2deg(pair.second));
                if (fake_wrist) {
                    ROS_WARN_THROTTLE(2.0, "Faking wrist.");
                    nh->setParam("wrist_right_angle", cmd.set_points[0]);
                } else {
                    motor_command_pub.publish(cmd);
                }
            }
        }
    }
    joint_target_pub.publish(js);
    return true;
}
