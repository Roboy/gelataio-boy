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
        : left_arm("left", 5), right_arm("right", 5), node_handle(handle), ice_box(nullptr), status("idle"),
          active_arm(&right_arm) {
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

        emo = new emotions_interface(handle);
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
        switch (active_arm->get_status()) {
            case HandController::IDLE:
                this->status = "IDLE";
                break;
            case HandController::PLANNING:
                this->status = "PLANNING";
                break;
            case HandController::EXECUTING:
                this->status = "EXECUTING";
                break;
            case HandController::ERROR:
                this->status = "ERROR";
                break;
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

    if (emo) emo->show_emotion("lucky");

    stringstream ss;
    ss << "Current pose of right arm: " << endl << right_arm.getCurrentPose() << endl;
    ROS_INFO_STREAM(ss.str());

    this->active_arm = &right_arm;
    ROS_INFO("Moving to the scene");
    bool successful = this->start_scoop_appraoch_via();

    if (successful) {
        ROS_INFO("Moving to start point for scooping");
        successful &= this->approach_scoop_point(start);
    } else {
        ROS_WARN("Skipping approach");
    }

    if (successful) {
        ROS_INFO("Movement to the start point was successful :)");
        ROS_INFO("Performing the scoop");
        if (emo) emo->show_emotion("surprise");
        successful &= this->perform_scoop();
    } else {
        ROS_WARN("Skipping perform scoop");
    }

     if (successful) {
         ROS_INFO("Departing from the scoop stage");
         successful &= this->depart_from_scoop();
     } else {
         ROS_WARN("Skipping scoop departing");
     }

     if (successful) {
         if (emo) emo->show_emotion("sunglasses_on");
         ROS_INFO("Dropping the ball");
         geometry_msgs::Point destination;
         destination.x = -0.4;
         destination.y = -0.3;
         destination.z = 0.4;
         successful &= this->drop_ice(destination);
     } else {
         ROS_WARN("Skipping the drop");
     }

     ros::Duration sleep(2.0);
     sleep.sleep();
     if (emo) emo->show_emotion("lucky");
     if (emo) emo->show_emotion("sunglasses_on");

     if (successful) {
         ROS_INFO("Scooping done without error");
     } else {
         ROS_ERROR("Scooping finished with error");
     }
    finish_cb(successful);
    return successful;
}

bool ScoopingMain::go_home(std::function<void(bool)> finish_cb) {

    stringstream ss;
    ss << "Current pose of right arm: " << endl << right_arm.getCurrentPose() << endl;
    ROS_INFO_STREAM(ss.str());
    bool successful = true;
    this->active_arm = &right_arm;

    ROS_INFO("Going home");
    successful &= right_arm.goHome();

    if (successful) {
        ROS_INFO("Succesfully went home.");
    } else {
        ROS_ERROR("Couldn't go home.");
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
    ice_boxPose.orientation.w = cos((tilt_angle / 2) / 180.0 * M_PI);
    ice_boxPose.orientation.x = sin((tilt_angle / 2) / 180.0 * M_PI);
    ice_boxPose.orientation.y = 0.0;
    ice_boxPose.orientation.z = 0.0;
    ice_box->primitive_poses.push_back(ice_boxPose);
    ice_box->operation = ice_box->ADD;
}

bool ScoopingMain::drop_ice(Point destination){
    if (cardsflow) {
        map<string, double> drop_ball_pose;

        drop_ball_pose["shoulder_right_axis0"] = -1.5;
        drop_ball_pose["shoulder_right_axis1"] = 0.9;
        drop_ball_pose["shoulder_right_axis2"] = 1.6;
        drop_ball_pose["elbow_right"] = 1.1;
        bool success = cardsflow->moveJointsTo(drop_ball_pose);
        ros::Duration wait(10.0);
        ROS_INFO("Waiting to reach approach via point");
        wait.sleep();

        success &= interpolate_joint("wrist_right", 0.4, 2.1, 5.0);
        return success;
    } else {
        Pose drop_approach;
        drop_approach.position = destination;

        moveit_msgs::Constraints c;
        moveit_msgs::JointConstraint dont_drop_ball_constraints;
        dont_drop_ball_constraints.joint_name = "wrist_right";
        dont_drop_ball_constraints.position = right_arm.jointStatus()["wrist_right"];
        dont_drop_ball_constraints.tolerance_below = .15;
        dont_drop_ball_constraints.tolerance_above = .15;
        dont_drop_ball_constraints.weight = 1.0;
        vector <string> constrained_shoulder_axes = {"shoulder_right_axis0", "shoulder_right_axis2"};
        map<string, double> joint_state = right_arm.jointStatus();
        for (const auto &ax : constrained_shoulder_axes) {
            ROS_INFO_STREAM("Setting constraint: " << ax << ": " << joint_state[ax]);
            moveit_msgs::JointConstraint ax_constraint;
            ax_constraint.joint_name = ax;
            ax_constraint.position = joint_state[ax];
            ax_constraint.tolerance_below = .3;
            ax_constraint.tolerance_above = .3;
            ax_constraint.weight = 1.0;
            c.joint_constraints.push_back(ax_constraint);
        }
        c.joint_constraints.push_back(dont_drop_ball_constraints);
        right_arm.setPlanningTime(10.0);

        bool success = right_arm.moveToPose(drop_approach, c, "bike_front");

        right_arm.setPlanningTime(1.0);
        success &= right_arm.moveJoint("wrist_right", 0.0);

        return success;
    }
}

bool ScoopingMain::start_scoop_appraoch_via() {
    map<string, double> via_point;

    via_point["shoulder_right_axis0"] = -1;
    via_point["shoulder_right_axis1"] = 1;
    via_point["shoulder_right_axis2"] = 1.6;
    via_point["elbow_right"] = 1.8;
    via_point["wrist_right"] = 2.8;
    bool result;
    if (cardsflow) {
      result = cardsflow->moveJointsTo(via_point);
      ros::Duration wait(10.0);
      ROS_INFO("Waiting to reach approach via point");
      wait.sleep();
    }
    else result =right_arm.moveJoints(via_point);

    return result;
}

bool ScoopingMain::approach_scoop_point(geometry_msgs::Point scoop_point) {
    if (cardsflow) {
        map<string, double> via_point;

        via_point["shoulder_right_axis0"] = -1;
        via_point["shoulder_right_axis1"] = 0.5;
        via_point["shoulder_right_axis2"] = 1.45;
        via_point["elbow_right"] = .95;
        via_point["wrist_right"] = 2.8;
        bool result;
        if (cardsflow) {
            result = cardsflow->moveJointsTo(via_point);
            ros::Duration wait(10.0);
            ROS_INFO("Waiting to reach approach via point");
            wait.sleep();
        } else result = right_arm.moveJoints(via_point);

        return result;
    } else {
        Pose scooping_start;
        scooping_start.position = scoop_point;
        tf2::Quaternion q_start;
        double roll = -20;
        double pitch = 40;
        double yaw = 45 + 90;
        q_start.setRPY(roll / 180 * M_PI, pitch / 180 * M_PI, yaw / 180 * M_PI);
        scooping_start.orientation.x = q_start.x();
        scooping_start.orientation.y = q_start.y();
        scooping_start.orientation.z = q_start.z();
        scooping_start.orientation.w = q_start.w();

        moveit_msgs::Constraints constraints;
        moveit_msgs::JointConstraint wristConstraint;
        wristConstraint.joint_name = "wrist_right";
        wristConstraint.position = 2.8;
        wristConstraint.tolerance_below = .1;
        wristConstraint.tolerance_above = 1.0;
        wristConstraint.weight = 1.0;
        constraints.joint_constraints.push_back(wristConstraint);

        vector<string> constrained_shoulder_axes = {"shoulder_right_axis0", "shoulder_right_axis2"};
        map<string, double> joint_state = right_arm.jointStatus();

        for (const auto &ax : constrained_shoulder_axes) {
            moveit_msgs::JointConstraint ax_constraint;
            ROS_INFO_STREAM("Setting constraint: " << ax << ": " <<joint_state[ax]);
            ax_constraint.joint_name = ax;
            ax_constraint.position = joint_state[ax];
            ax_constraint.tolerance_below = .3;
            ax_constraint.tolerance_above = .3;
            ax_constraint.weight = 1.0;
            constraints.joint_constraints.push_back(ax_constraint);
        }
        right_arm.setPlanningTime(10.0);

        if (cardsflow) cardsflow->moveJointTo("wrist_right", wristConstraint.position);
        return right_arm.moveToPose(scooping_start, constraints);

    }

}

bool ScoopingMain::perform_scoop() {
    right_arm.setPlanningTime(1.0);
    if (cardsflow) return this->interpolate_joint("wrist_right", right_arm.jointStatus()["wrist_right"], 0.4, 10.0);
    else return right_arm.moveJoint("wrist_right", 0.4);
}

bool ScoopingMain::depart_from_scoop() {
    if (cardsflow) {
        std::map<std::string, double> depart_pose;
        depart_pose["shoulder_right_axis0"] = -1.3;
        depart_pose["shoulder_right_axis1"] = 0.8;
        depart_pose["shoulder_right_axis2"] = 1.7;
        depart_pose["elbow_right"] = 1.8;
        bool result;
        result = cardsflow->moveJointsTo(depart_pose);
        ros::Duration wait(3.0);
        ROS_INFO("Departing");
        wait.sleep();
        return result;
    } else {
        Pose current_pose = right_arm.getCurrentPose().pose;
        Pose move_up(current_pose);
        move_up.position.z += 0.1;

        moveit_msgs::Constraints c;
        moveit_msgs::JointConstraint dont_drop_ball_constraints;
        dont_drop_ball_constraints.joint_name = "wrist_right";
        dont_drop_ball_constraints.position = right_arm.jointStatus()["wrist_right"];
        dont_drop_ball_constraints.tolerance_below = .25;
        dont_drop_ball_constraints.tolerance_above = .25;
        dont_drop_ball_constraints.weight = 1.0;
        c.joint_constraints.push_back(dont_drop_ball_constraints);
        vector <string> constrained_shoulder_axes = {"shoulder_right_axis0", "shoulder_right_axis2"};
        map<string, double> joint_state = right_arm.jointStatus();
        for (const auto &ax : constrained_shoulder_axes) {
            ROS_INFO_STREAM("Setting constraint: " << ax << ": " << joint_state[ax]);
            moveit_msgs::JointConstraint ax_constraint;
            ax_constraint.joint_name = ax;
            ax_constraint.position = joint_state[ax];
            ax_constraint.tolerance_below = .3;
            ax_constraint.tolerance_above = .3;
            ax_constraint.weight = 1.0;
            c.joint_constraints.push_back(ax_constraint);
        }
        right_arm.setPlanningTime(10.0);

        return right_arm.moveToPose(move_up, c);
    }
}


bool ScoopingMain::init_pose(std::function<void(bool)> finish_cb) {

    map<string, double> init_pose;
    init_pose["shoulder_right_axis0"] = 0.0;
    init_pose["shoulder_right_axis1"] = 0.6;
    init_pose["shoulder_right_axis2"] = 0.0;
    init_pose["elbow_right"] = 0.0;
    init_pose["wrist_right"] = 0.0;

    bool successful;

    if (cardsflow) {
            successful = cardsflow->moveJointsTo(init_pose);
            ros::Duration s(3.0);
            s.sleep();
    } else successful = right_arm.moveJoints(init_pose);

    finish_cb(successful);
    return successful;
}

bool ScoopingMain::interpolate_joint(std::string joint_name, double from, double to, double time, int steps) {
    ros::Duration dt(time / ((double) steps));
    for (int i = 0; i < steps; i++) {
        cardsflow->moveJointTo(joint_name, from + (to - from)*((double)i/steps));
        dt.sleep();
    }
    return true;
}
