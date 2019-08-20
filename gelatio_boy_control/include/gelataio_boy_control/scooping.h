//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_H
#define SRC_SCOOPING_H


#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "hand_controller.hpp"

class ScoopingMain {
public:
    explicit ScoopingMain(ros::NodeHandle *node_handle);

    virtual ~ScoopingMain();

    void scoop_ice();

    std::string get_status();


    //////////////////////////////////////////////////

    void hello();


protected:
    virtual void defineEnvironment();


private:
    HandController right_arm, left_arm;
    ros::NodeHandle *node_handle;

    CardsflowPlanExecutor *right_cardsflow, *left_cardsflow;

    hand_interface *right_hand, *left_hand;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::string status;
};


#endif //SRC_SCOOPING_H
