//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping.h"

ScoopingMain::ScoopingMain(ros::NodeHandle *handle) : left_arm("left", 5), right_arm("right", 5), node_handle(handle), status("") {
    left_cardsflow = new CardsflowPlanExecutor("left", handle);
    left_hand = new DummyHand("left hand");
    left_arm.addPlanExecutor(left_cardsflow);
    left_arm.setHandInterface(left_hand);

    right_cardsflow = new CardsflowPlanExecutor("right", handle);
    right_hand = new DummyHand("right hand");
    right_arm.addPlanExecutor(right_cardsflow);
    right_arm.setHandInterface(right_hand);
}

ScoopingMain::~ScoopingMain() {
    delete left_cardsflow;
    delete right_cardsflow;
    delete left_hand;
    delete right_hand;

}

void ScoopingMain::hello() {
    right_arm.moveToKnownPose("hello_start");
    right_arm.moveToKnownPose("hello_end");
}

void ScoopingMain::defineEnvironment() {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // -------------------- Table -----------------------
    collision_objects[0].id = "ice_cream_table";
    collision_objects[0].header.frame_id = "torso";

    // Define the primitive and its dimensions.
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.8;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.3;

    // Define the pose of the table.
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0;
    collision_objects[0].primitive_poses[0].position.y = -0.5;
    collision_objects[0].primitive_poses[0].position.z = 0.01;

    collision_objects[0].operation = collision_objects[0].ADD;

    // -------------------- Cup -----------------------
    collision_objects[1].id = "ice_cup";
    collision_objects[1].header.frame_id = "torso";

    // Define the primitive and its dimensions.
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
    collision_objects[1].primitives[0].dimensions.resize(2);
    collision_objects[1].primitives[0].dimensions[0] = 0.1;
    collision_objects[1].primitives[0].dimensions[1] = 0.05;

    // Define the pose of the table.
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = -0.2;
    collision_objects[1].primitive_poses[0].position.y = -0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.4;

    collision_objects[1].operation = collision_objects[1].ADD;

    // -------------------- Scooper -----------------------
    collision_objects[2].id = "ice_scooper";
    collision_objects[2].header.frame_id = "torso";

    // Define the primitive and its dimensions.
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
    collision_objects[2].primitives[0].dimensions.resize(2);
    collision_objects[2].primitives[0].dimensions[0] = 0.2;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;

    // Define the pose of the table.
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.2;
    collision_objects[2].primitive_poses[0].position.y = -0.5;
    collision_objects[2].primitive_poses[0].position.z = 0.4;

    collision_objects[2].operation = collision_objects[1].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

void ScoopingMain::scoop_ice() {
    geometry_msgs::Pose cup_pose;
    cup_pose.position.x = -0.27;
    cup_pose.position.y = -0.45;
    right_arm.grasp("ice_cup", cup_pose);

    geometry_msgs::Pose scooper_pose;
    scooper_pose.position.x = 0.24;
    scooper_pose.position.y = -0.45;

    left_arm.grasp("ice_scooper", scooper_pose);

    left_arm.moveToKnownPose("ready_to_scoope");

    left_arm.moveToKnownPose("scoope_1");
    left_arm.moveToKnownPose("scoope_2");

    left_arm.moveToKnownPose("ready_to_grab");

    geometry_msgs::PoseStamped receiving_ball_cup_pose = right_arm.getCurrentPose();
    receiving_ball_cup_pose.pose.position.x += 0.25;
    receiving_ball_cup_pose.pose.position.z -= 0.1;
    right_arm.moveToPose(receiving_ball_cup_pose);

    geometry_msgs::PoseStamped drop_ball_pose = left_arm.getCurrentPose();
    drop_ball_pose.pose.position.x -= 0.15;
    drop_ball_pose.pose.position.z += 0.1;
    left_arm.moveToPose(drop_ball_pose);

    drop_ball_pose.pose.position.x -= 0.1;
    drop_ball_pose.pose.orientation.w = -0.707;
    drop_ball_pose.pose.orientation.x = 0.0;
    drop_ball_pose.pose.orientation.y = 0.707;
    drop_ball_pose.pose.orientation.z = 0;
    left_arm.moveToPose(drop_ball_pose);
}

