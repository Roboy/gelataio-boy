//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_H
#define SRC_SCOOPING_H


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include "hand_controller.hpp"
#include "emotions_interface.h"

class ScoopingMain {
public:
    explicit ScoopingMain(ros::NodeHandle *node_handle, bool simulation_only = false);

    virtual ~ScoopingMain();

    bool scoop_ice(geometry_msgs::Point start, geometry_msgs::Point end, std::function<void(bool)> finish_cb);
    bool go_home(std::function<void(bool)> finish_cb);
    bool init_pose(std::function<void(bool)> finish_cb);
    std::string get_status() {return status;}


    void hello();


protected:
    bool start_scoop_appraoch_via();
    bool approach_scoop_point(geometry_msgs::Point scoop_point);
    bool perform_scoop();
    bool depart_from_scoop();
    bool drop_ice(geometry_msgs::Point destination);

    bool interpolate_joint(std::string joint_name, double from, double to, double time, int steps = 50);

    virtual void defineEnvironment();

    virtual void createObstacles();

    void watch_status();


private:
    HandController right_arm, left_arm;
    ros::NodeHandle *node_handle;

    CardsflowPlanExecutor *cardsflow;

    HandController* active_arm;

    hand_interface *right_hand, *left_hand;

    emotions_interface *emo;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string status;

    std::shared_ptr<std::thread> status_watcher;


    //obstacles
    moveit_msgs::CollisionObject *ice_box;
};


#endif //SRC_SCOOPING_H
