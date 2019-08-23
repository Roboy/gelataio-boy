//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_H
#define SRC_SCOOPING_H


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include "hand_controller.hpp"

class ScoopingMain {
public:
    explicit ScoopingMain(ros::NodeHandle *node_handle, bool simulation_only = false);

    virtual ~ScoopingMain();

    bool scoop_ice(geometry_msgs::Point start, geometry_msgs::Point end, std::function<void(bool)> finish_cb);
    std::string get_status() {return status;}


    void hello();


protected:
    bool approach_scoop_point(geometry_msgs::Point scoop_point);
    bool perform_scoop(geometry_msgs::Point end_point);
    bool depart_from_scoop();
    bool drop_ice(geometry_msgs::Point destination);

    virtual void defineEnvironment();

    virtual void createObstacles();

    void watch_status();


private:
    HandController right_arm, left_arm;
    ros::NodeHandle *node_handle;

    CardsflowPlanExecutor *right_cardsflow, *left_cardsflow;

    HandController* active_arm;

    hand_interface *right_hand, *left_hand;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string status;

    std::shared_ptr<std::thread> status_watcher;


    //obstacles
    moveit_msgs::CollisionObject *ice_box;
};


#endif //SRC_SCOOPING_H
