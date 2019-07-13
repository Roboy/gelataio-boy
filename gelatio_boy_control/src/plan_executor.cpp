//
// Created by arne on 13.07.19.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <gelataio_boy_control/plan_executor.h>

bool MoveItPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    moveit::planning_interface::MoveItErrorCode execution_result = this->move_it->execute(plan);
    return execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

CardsflowPlanExecutor::CardsflowPlanExecutor(std::string &group_name) : group_name(group_name) {

}

bool CardsflowPlanExecutor::executePlan(moveit::planning_interface::MoveGroupInterface::Plan &plan) {
    std::cout << "Moving " << group_name << " using CARDSflow to:" << std::endl;
    std::cout << plan.trajectory_ << std::endl;
    return true;
}
