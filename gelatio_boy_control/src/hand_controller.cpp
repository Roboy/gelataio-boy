#include "gelataio_boy_control/hand_controller.hpp"

HandController::HandController(std::string planning_group, int planning_attempts, PlanningExecutorMode mode) {
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

HandController::PlanningResult HandController::plan() {
    this->m_move_group_ptr->setStartStateToCurrentState();

    ROS_INFO_NAMED("HandController", "HandController::Planning frame: %s",
                   this->m_move_group_ptr->getPlanningFrame().c_str());
    ROS_INFO_NAMED("HandController", "HandController::End effector link: %s",
                   this->m_move_group_ptr->getEndEffectorLink().c_str());

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode planning_result = this->m_move_group_ptr->plan(plan);
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

    return this->m_plan_executor_ptr->executePlan(planning_result.plan);
}

bool HandController::moveToPose(geometry_msgs::PoseStamped target_pose) {
    this->m_move_group_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPose(geometry_msgs::Pose target_pose) {
    this->m_move_group_ptr->setPoseTarget(target_pose);

    return this->planAndExecute();
}

bool HandController::moveToPoses(std::vector<geometry_msgs::Pose> &targets) {
    this->m_move_group_ptr->setPoseTargets(targets);

    return this->planAndExecute();
}

bool HandController::moveToPosition(geometry_msgs::Point target_position) {
    geometry_msgs::PoseStamped target_pose_st = this->getCurrentPose();
    ROS_ERROR_STREAM("CURRENT POSE " << target_pose_st.header.frame_id);
    geometry_msgs::Pose target_pose = target_pose_st.pose;
    ROS_ERROR_STREAM("CURRENT POSE " << target_pose.position.x << " " << target_pose.position.y << " "
                                     << target_pose.position.z);
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

void HandController::grasp(std::string object_name) {

    this->moveToKnownPose("ready_to_grab");

    // TODO: figure out how to move properly the palm to the cup
    geometry_msgs::Point target_pose = this->getCurrentPose().pose.position;
    target_pose.x -= 0.05;

    this->moveToPosition(target_pose);

    // TODO: part of pick and place tutorial - adjust to Roboy
    /*std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "torso";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = -0.1;
    grasps[0].grasp_pose.pose.position.y = -0.3;
    grasps[0].grasp_pose.pose.position.z = 0.55;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    grasps[0].pre_grasp_approach.direction.header.frame_id = "torso";
    grasps[0].pre_grasp_approach.direction.vector.x = -1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    grasps[0].post_grasp_retreat.direction.header.frame_id = "torso";
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
    
    // +++++++++++++++++++++++++++++++++++
    //openGripper(grasps[0].pre_grasp_posture);

    grasps[0].pre_grasp_posture.joint_names.resize(2);
    grasps[0].pre_grasp_posture.joint_names[0] = "hand_right_ring_joint0";
    grasps[0].pre_grasp_posture.joint_names[1] = "hand_right_thumb_joint0";

    grasps[0].pre_grasp_posture.points.resize(1);
    grasps[0].pre_grasp_posture.points[0].positions.resize(2);
    grasps[0].pre_grasp_posture.points[0].positions[0] = 0.04;
    grasps[0].pre_grasp_posture.points[0].positions[1] = 0.04;
    grasps[0].pre_grasp_posture.points[0].time_from_start = ros::Duration(0.5);
    
    // +++++++++++++++++++++++++++++++++++
    //closedGripper(grasps[0].grasp_posture);


    this->m_move_group_ptr->setSupportSurfaceName("ice_cream_table");

    this->m_move_group_ptr->pick(object_name, grasps);*/

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