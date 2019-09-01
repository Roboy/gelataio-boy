//
// Created by arne on 13.07.19.
//

#ifndef SRC_PLAN_EXECUTOR_H
#define SRC_PLAN_EXECUTOR_H


class plan_executor {
public:
    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) = 0;
};

class CardsflowPlanExecutor : public plan_executor {
public:
    explicit CardsflowPlanExecutor(ros::NodeHandle *nh);

    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) override;

    virtual bool moveJointTo(std::string joint_name, double target);

private:
    ros::Publisher joint_target_pub;
    ros::Publisher motor_command_pub;
    std::vector<std::string> ignored_joints;
};

#endif //SRC_PLAN_EXECUTOR_H
