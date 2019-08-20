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
        : left_arm("left", 5), right_arm("right", 5), node_handle(handle), status("") {
    if (simulation_only) { //Simulation
        left_hand = new DummyHand("left hand");
        left_arm.setHandInterface(left_hand);
        right_hand = new DummyHand("right hand");
        right_arm.setHandInterface(right_hand);
    } else { // Execution on Hardware
        left_cardsflow = new CardsflowPlanExecutor("left", handle);
        left_hand = new DummyHand("left hand");
        left_arm.addPlanExecutor(left_cardsflow);
        left_arm.setHandInterface(left_hand);

        right_cardsflow = new CardsflowPlanExecutor("right", handle);
        right_hand = new DummyHand("right hand");
        right_arm.addPlanExecutor(right_cardsflow);
        right_arm.setHandInterface(right_hand);
    }

    this->createObstacles();

    this->right_hand->grasp(); // apply force on the scooper
}

ScoopingMain::~ScoopingMain() {
    delete left_cardsflow;
    delete right_cardsflow;
    delete left_hand;
    delete right_hand;

    delete ice_box;

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

void ScoopingMain::scoop_ice(Point &start, Point &end) {
    this->defineEnvironment();

    auto &scooping_arm = right_arm;

    Pose scooping_start, scooping_end;

    scooping_start.position = start;
    tf2::Quaternion q_start;
    double roll = -20;
    double pitch = 40;
    double yaw = 45 + 90;
    q_start.setRPY(roll/180*M_PI, pitch/180*M_PI, yaw/180*M_PI);
    scooping_start.orientation.x = q_start.x();
    scooping_start.orientation.y = q_start.y();
    scooping_start.orientation.z = q_start.z();
    scooping_start.orientation.w = q_start.w();

    scooping_end.position = end;
    scooping_end.orientation = scooping_start.orientation;

    scooping_arm.moveToPose(scooping_start);
//    scooping_arm.moveToPose(scooping_end);


    //TODO add the depart motion

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

void ScoopingMain::drop_ice(Point &destination) {
    ROS_ERROR("Not implemented.");

}

