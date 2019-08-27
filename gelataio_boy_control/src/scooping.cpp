//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping.h"

using namespace moveit;
using namespace moveit_msgs;
using namespace shape_msgs;
using namespace geometry_msgs;
using namespace std;

ScoopingMain::ScoopingMain(ros::NodeHandle *handle, bool simulation_only)
        : left_arm("left", 5), right_arm("right", 5), node_handle(handle), ice_box(nullptr), status("idle"), active_arm(&right_arm) {
    if (simulation_only) { //Simulation
        left_hand = new DummyHand("left hand");
        left_arm.setHandInterface(left_hand);
        right_hand = new DummyHand("right hand");
        right_arm.setHandInterface(right_hand);
    } else { // Execution on Hardware
        cardsflow = new CardsflowPlanExecutor(handle);
        left_hand = new DummyHand("left hand");
        left_arm.addPlanExecutor(cardsflow);
        left_arm.setHandInterface(left_hand);

        right_hand = new DummyHand("right hand");
        right_arm.addPlanExecutor(cardsflow);
        right_arm.setHandInterface(right_hand);
    }

    this->createObstacles();

    this->status_watcher = make_shared<thread>(&ScoopingMain::watch_status, this);

    this->right_hand->grasp(); // apply force on the scooper
}

ScoopingMain::~ScoopingMain() {
    this->status_watcher->join();
    delete cardsflow;
    delete left_hand;
    delete right_hand;

    delete ice_box;
}

void ScoopingMain::watch_status() {
    ros::Rate r(10.0);
    while (ros::ok()) {
        switch(active_arm->get_status()) {
            case HandController::IDLE: this->status = "IDLE"; break;
            case HandController::PLANNING: this->status = "PLANNING"; break;
            case HandController::EXECUTING: this->status = "EXECUTING"; break;
            case HandController::ERROR: this->status = "ERROR"; break;
        }
    }
}

void ScoopingMain::hello() {
    right_arm.moveToKnownPose("hello_start");
    right_arm.moveToKnownPose("hello_end");
}

void ScoopingMain::defineEnvironment() {
    std::vector<moveit_msgs::CollisionObject> collision_objects;

    collision_objects.push_back(*ice_box);

    // ------------------------------------------------------
    stringstream ss;
    ss << "Adding these obstacles to the scene:" << endl;
    for (const auto &item : collision_objects) {
        ss << "\t" << item.id << endl;
    }
    ROS_INFO_STREAM(ss.str());
    planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool ScoopingMain::scoop_ice(Point start, Point end, std::function<void(bool)> finish_cb) {
//    this->defineEnvironment();

    stringstream ss; ss << "Current pose of right arm: " << endl << right_arm.getCurrentPose() << endl;
    ROS_INFO_STREAM(ss.str());

    this->active_arm = &right_arm;
    ROS_INFO("Moving to start point for scooping");
    bool successful = this->approach_scoop_point(start);
    if (successful) {
        ROS_INFO("Movement to the start point was successful :)");
        ROS_INFO("Performing the scoop");
        successful &= this->perform_scoop();
    }

    if (successful) {
        ROS_INFO("Departing from the scoop stage");
        successful &= this->depart_from_scoop();
    }

    ROS_INFO("Going home");
    successful &= right_arm.goHome();

    if (successful) {
        ROS_INFO("Scooping done without error");
    } else {
        ROS_ERROR("Scooping finished with error");
    }
    finish_cb(successful);
    return successful;
}

void ScoopingMain::createObstacles() {

    // --------------------- Ice Cream Box ------------------
    ice_box = new CollisionObject();
    ice_box->id = "ice_cream_ice_box";
    ice_box->header.frame_id = "torso";

    SolidPrimitive solid_ice_box;
    solid_ice_box.type = solid_ice_box.BOX;
    solid_ice_box.dimensions.resize(3);
    solid_ice_box.dimensions[0] = 0.6;
    solid_ice_box.dimensions[1] = 0.6;
    solid_ice_box.dimensions[2] = 0.6;
    ice_box->primitives.push_back(solid_ice_box);

    Pose ice_boxPose;
    ice_boxPose.position.x = 0;
    ice_boxPose.position.y = -0.7;
    ice_boxPose.position.z = 0.2;
    const double tilt_angle = -20.0; // deg
    ice_boxPose.orientation.w = cos((tilt_angle/2) / 180.0 * M_PI);
    ice_boxPose.orientation.x = sin((tilt_angle/2) / 180.0 * M_PI);
    ice_boxPose.orientation.y = 0.0;
    ice_boxPose.orientation.z = 0.0;
    ice_box->primitive_poses.push_back(ice_boxPose);
    ice_box->operation = ice_box->ADD;
}

bool ScoopingMain::approach_scoop_point(geometry_msgs::Point scoop_point) {
    Pose scooping_start;

    scooping_start.position = scoop_point;
    tf2::Quaternion q_start;
    double roll = -20;
    double pitch = 40;
    double yaw = 45 + 90;
    q_start.setRPY(roll/180*M_PI, pitch/180*M_PI, yaw/180*M_PI);
    scooping_start.orientation.x = q_start.x();
    scooping_start.orientation.y = q_start.y();
    scooping_start.orientation.z = q_start.z();
    scooping_start.orientation.w = q_start.w();

    moveit_msgs::Constraints constraints;
    moveit_msgs::JointConstraint wristConstraint;
    wristConstraint.joint_name = "wrist_right";
    wristConstraint.position = 0.8;
    wristConstraint.tolerance_below = .1;
    wristConstraint.tolerance_above = 3.14/2;
    wristConstraint.weight = 1.0;
    constraints.joint_constraints.push_back(wristConstraint);
    right_arm.setPlanningTime(10.0);
    return right_arm.moveToPose(scooping_start, constraints);
}

bool ScoopingMain::perform_scoop() {
    right_arm.setPlanningTime(1.0);
    return right_arm.moveJoint("wrist_right", -0.5);
}

bool ScoopingMain::drop_ice() {
    return right_arm.moveJoint("wrist_right", 1.0);
}

bool ScoopingMain::depart_from_scoop(geometry_msgs::Point point_above_cup) {
    right_arm.setPlanningTime(5.0);
    Pose hold_scoop_pose;
    hold_scoop_pose.position = point_above_cup;
    moveit_msgs::JointConstraint wristConstraint;
    bool scooper_hold = false;
    bool drop_success = false;

    wristConstraint.joint_name = "wrist_right";
    wristConstraint.position = 0.;
    wristConstraint.tolerance_below = .1;
    wristConstraint.tolerance_above = .1;
    wristConstraint.weight = 1.0;
    
    // Wrist right go to zero and then it should nevere move again
    // We should add the box as a constrain
    scooper_hold = right_arm.moveToPose(hold_scoop_pose, wristConstraint);

    if (!scooper_hold){
        ROS_ERROR("Scooper could spill icecream");
        return false;
    }

    drop_success = drop_ice();

    if (!drop_success){
        ROS_ERROR("Could not frop icecream");
        return false;
    }

    return true;
}

