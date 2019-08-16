#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "gelataio_boy_control/hand_controller.hpp"

void defineEnvironment(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface) {
    ROS_INFO("SCOOPING PLANNING");
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

int main(int argc, char **argv) {
    ros::init(argc, argv, "gelataio_arm_controller");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    HandController right_arm("right", 5);
    CardsflowPlanExecutor right_arm_cardsflow("right", &node_handle);
    right_arm.addPlanExecutor(&right_arm_cardsflow);

    HandController left_arm("left", 5);
    CardsflowPlanExecutor left_arm_cardsflow("left", &node_handle);
    left_arm.addPlanExecutor(&left_arm_cardsflow);
    
    defineEnvironment(planning_scene_interface);

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

    while (true) {
        right_arm.moveToKnownPose("hello_start");
        right_arm.moveToKnownPose("hello_end");
    }

    return 0;
}
