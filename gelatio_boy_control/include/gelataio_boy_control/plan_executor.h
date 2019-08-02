//
// Created by arne on 13.07.19.
//

#ifndef SRC_PLAN_EXECUTOR_H
#define SRC_PLAN_EXECUTOR_H

enum PlanningExecutorMode {
    MOVE_IT = 1,
    CARDSFLOW = 2
};

class plan_executor {
public:
    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) = 0;
};

class MoveItPlanExecutor : public plan_executor {

public:
    explicit MoveItPlanExecutor(moveit::planning_interface::MoveGroupInterface *move_it) : move_it(move_it) {}

    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) override;

private:
    moveit::planning_interface::MoveGroupInterface *move_it;
};

class CardsflowPlanExecutor : public plan_executor {
public:
    explicit CardsflowPlanExecutor(std::string &group_name, ros::NodeHandle *nh, moveit::planning_interface::MoveGroupInterface *move_it);

    virtual bool executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) override;

private:
    std::string group_name;
    std::map<std::string, ros::Publisher> publishers;
    MoveItPlanExecutor mipe;
};

#endif //SRC_PLAN_EXECUTOR_H
