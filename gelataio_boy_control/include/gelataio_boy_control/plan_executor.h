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
    explicit CardsflowPlanExecutor(std::string group_name, ros::NodeHandle *nh);

    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) override;

private:
    std::string group_name;
    ros::Publisher joint_target_pub;
};

#endif //SRC_PLAN_EXECUTOR_H
