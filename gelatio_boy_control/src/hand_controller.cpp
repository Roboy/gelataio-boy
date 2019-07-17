#include "gelataio_boy_control/hand_controller.hpp"

HandController::HandController(std::string planning_group_hand, std::string planning_group_arm, int planning_attempts) {
    this->m_planning_group_hand = planning_group_hand;
    this->m_planning_group_arm = planning_group_arm;

    this->m_move_group_hand_ptr = new moveit::planning_interface::MoveGroupInterface(planning_group_hand);
    this->m_move_group_arm_ptr = new moveit::planning_interface::MoveGroupInterface(planning_group_arm);

    this->m_planning_attempts = planning_attempts;
}

HandController::PlanningResult HandController::plan(enum HandController::PlanningGroups planning_gr, double tolerance) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode planning_result;

    if (planning_gr == HandController::PlanningGroups::ARM) {
        this->m_move_group_arm_ptr->setStartStateToCurrentState();
        this->m_move_group_arm_ptr->setGoalTolerance(tolerance);
        planning_result = this->m_move_group_arm_ptr->plan(plan); 
    }
    else if (planning_gr == HandController::PlanningGroups::HAND) {
        this->m_move_group_hand_ptr->setStartStateToCurrentState();
        this->m_move_group_hand_ptr->setGoalTolerance(tolerance);
        planning_result = this->m_move_group_hand_ptr->plan(plan);  
    }

    return HandController::PlanningResult{plan, planning_result};
}

bool HandController::planAndExecute(enum HandController::PlanningGroups planning_gr) {
    HandController::PlanningResult planning_result = this->plan(planning_gr);

    int attempts_counter = 0;
    bool path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;

    while (attempts_counter <= this->m_planning_attempts && !path_found) {
        planning_result = this->plan(planning_gr);

        path_found = planning_result.planning_status == moveit::planning_interface::MoveItErrorCode::SUCCESS;
        attempts_counter++;
    }

    moveit::planning_interface::MoveItErrorCode execution_result = this->m_move_group_arm_ptr->execute(
            planning_result.plan);
    return execution_result == moveit::planning_interface::MoveItErrorCode::SUCCESS;
}

bool HandController::moveToPose(geometry_msgs::PoseStamped target_pose) {
    this->m_move_group_arm_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPose(geometry_msgs::Pose target_pose) {
    //this->m_move_group_arm_ptr->setWorkspace(-.5, -1, .3, .5, 1., 1.);

    this->m_move_group_arm_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPoses(std::vector<geometry_msgs::Pose> &targets) {
    this->m_move_group_arm_ptr->setPoseTargets(targets);

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

bool HandController::moveToKnownPose(std::string pose_name, enum HandController::PlanningGroups planning_gr) {
    if (planning_gr == HandController::PlanningGroups::ARM) {
        this->m_move_group_arm_ptr->setNamedTarget(pose_name);  
    }
    else if (planning_gr == HandController::PlanningGroups::HAND) {
        this->m_move_group_hand_ptr->setNamedTarget(pose_name); 
    }

    return this->planAndExecute();
}

void HandController::grasp(std::string object_name, geometry_msgs::Pose target_pose) {

    this->moveToKnownPose("ready_to_grab", HandController::PlanningGroups::ARM);

    // TODO: figure out how to move properly the palm to the cup
    geometry_msgs::Point adjusted_pose = this->getCurrentPose().pose.position;
    adjusted_pose.x = target_pose.position.x;
    
    this->moveToPosition(adjusted_pose);

    this->m_move_group_hand_ptr->attachObject(object_name);

    this->moveToKnownPose("ready_to_grab", HandController::PlanningGroups::ARM);

}