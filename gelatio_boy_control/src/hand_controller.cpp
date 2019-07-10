#include "gelataio_boy_control/hand_controller.hpp"

HandController::HandController(std::string planning_group, int planning_attempts, PlanningExecutorMode mode)
{
    this->m_planning_group = planning_group;
    this->m_move_group_ptr = new moveit::planning_interface::MoveGroupInterface(planning_group);
    this->m_planning_attempts = planning_attempts;

    switch (mode) {
      case PlanningExecutorMode::MOVE_IT:
        this->m_plan_executor_ptr = new MoveItPlanExecutor(m_move_group_ptr);
        break;
      case PlanningExecutorMode::CARDSFLOW:
        this->m_plan_executor_ptr = new CardsflowPlanExecutor(planning_group);
        break;
    }
}

HandController::~HandController() {
  delete this->m_move_group_ptr;
  delete this->m_plan_executor_ptr;
}

HandController::PlanningResult HandController::plan()
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode planning_result = this->m_move_group_ptr->plan(plan);
    return HandController::PlanningResult{plan, planning_result};
}

bool HandController::planAndExecute()
{
    HandController::PlanningResult planning_result = this->plan();

    int attempts_counter = 0;
    bool path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    while(attempts_counter <= this->m_planning_attempts && !path_found)
    {
        planning_result = this->plan();

        path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        attempts_counter++;
    }

    return this->m_plan_executor_ptr->executePlan(planning_result.plan);
}

bool HandController::moveToPose(geometry_msgs::PoseStamped target_pose)
{
    this->m_move_group_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPose(geometry_msgs::Pose target_pose)
{
    this->m_move_group_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPoses(std::vector<geometry_msgs::Pose> &targets)
{
    this->m_move_group_ptr->setPoseTargets(targets);

    return this->planAndExecute();
}

bool HandController::moveToPosition(geometry_msgs::Point target_position)
{
    geometry_msgs::Pose target_pose = this->getCurrentPose().pose;
    target_pose.position = target_position;

    return this->moveToPose(target_pose);
}

bool HandController::moveToOrientation(geometry_msgs::Quaternion target_orientation)
{
    geometry_msgs::Pose target_pose = this->getCurrentPose().pose;
    target_pose.orientation = target_orientation;

    return this->moveToPose(target_pose);
}

bool HandController::moveToKnownPose(std::string pose_name)
{
    this->m_move_group_ptr->setNamedTarget(pose_name);

    return this->planAndExecute();
}

void HandController::grasp()
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

}

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
