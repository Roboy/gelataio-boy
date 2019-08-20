//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_H
#define SRC_SCOOPING_H


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "hand_controller.hpp"

class ScoopingMain {
public:
    explicit ScoopingMain(ros::NodeHandle *node_handle, bool simulation_only = false);

    virtual ~ScoopingMain();

    void scoop_ice(geometry_msgs::Point &start, geometry_msgs::Point &end);
    void drop_ice(geometry_msgs::Point &destination);

    std::string get_status();


    void hello();


//protected:
    virtual void defineEnvironment();

    virtual void createObstacles();


private:
    HandController right_arm, left_arm;
    ros::NodeHandle *node_handle;

    CardsflowPlanExecutor *right_cardsflow, *left_cardsflow;

    hand_interface *right_hand, *left_hand;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string status;


    //obstacles
    moveit_msgs::CollisionObject *ice_box;
};


#endif //SRC_SCOOPING_H
