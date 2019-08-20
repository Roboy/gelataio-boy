#include "gelataio_boy_control/hand_controller.hpp"

HandController::HandController(const std::string& group_name, int planning_attempts) : m_plan_executor_ptr(nullptr) {
    this->m_move_group_ptr = new moveit::planning_interface::MoveGroupInterface(group_name + "_arm");
    this->m_planning_attempts = planning_attempts;

}

HandController::PlanningResult HandController::plan(double tolerance) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode planning_result;

    this->m_move_group_ptr->setStartStateToCurrentState();
    this->m_move_group_ptr->setGoalTolerance(tolerance);
    planning_result = this->m_move_group_ptr->plan(plan);

    return HandController::PlanningResult{plan, planning_result};
}

bool HandController::planAndExecute() {
    HandController::PlanningResult planning_result = this->plan();

    int attempts_counter = 0;
    bool path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    while (attempts_counter <= this->m_planning_attempts && !path_found) {
        planning_result = this->plan();

        path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        attempts_counter++;
    }

    if (path_found) {
        if (this->m_plan_executor_ptr) {
            this->m_plan_executor_ptr->executePlan(planning_result.plan);
        }
        return this->m_move_group_ptr->execute(planning_result.plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    } else {
        ROS_ERROR("No plan found");
        return false;
    }
}

bool HandController::moveToPose(geometry_msgs::PoseStamped target_pose) {
    this->m_move_group_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPose(geometry_msgs::Pose target_pose) {
    std::stringstream ss;
    ss << "Target pose for moveIT planning: " << std::endl << target_pose << std::endl;
    ROS_INFO_STREAM(ss.str());

    this->m_move_group_ptr->setPoseTarget(target_pose);
    return this->planAndExecute();
}

bool HandController::moveToPoses(std::vector<geometry_msgs::Pose> &targets) {
    this->m_move_group_ptr->setPoseTargets(targets);

    return this->planAndExecute();
}

bool HandController::moveToPosition(geometry_msgs::Point target_position) {
    geometry_msgs::Pose target_pose = this->getCurrentPose().pose;
    target_pose.position = target_position;

    return this->moveToPose(target_pose);
}

bool HandController::moveToOrientation(geometry_msgs::Quaternion target_orientation) {
    geometry_msgs::Pose target_pose = this->getCurrentPose().pose;
    target_pose.orientation = target_orientation;

    return this->moveToPose(target_pose);
}

bool HandController::moveToKnownPose(std::string pose_name) {
    this->m_move_group_ptr->setNamedTarget(pose_name);

    return this->planAndExecute();
}

void HandController::grasp(std::string object_name, geometry_msgs::Pose target_pose) {
    this->moveToKnownPose("ready_to_grab");

    // TODO: figure out how to move properly the palm to the cup
    geometry_msgs::Point adjusted_pose = this->getCurrentPose().pose.position;
    adjusted_pose.x = target_pose.position.x;

    this->moveToPosition(adjusted_pose);
    if (this->m_hand_interface_ptr) {
        this->m_hand_interface_ptr->grasp();
    }

}

geometry_msgs::PoseStamped HandController::getCurrentPose() {
    return this->m_move_group_ptr->getCurrentPose();
}

void HandController::addPlanExecutor(plan_executor *executor) {
    this->m_plan_executor_ptr = executor;
}

void HandController::setHandInterface(hand_interface *interface) {
    this->m_hand_interface_ptr = interface;
}

