//
// Created by arne on 20.08.19.
//

#ifndef SRC_SCOOPING_H
#define SRC_SCOOPING_H


#include "hand_controller.hpp"

class ScoopingMain {
public:
    ScoopingMain(ros::NodeHandle *node_handle);

    virtual ~ScoopingMain();


private:
    HandController right_arm, left_arm;
    ros::NodeHandle *node_handle;

    CardsflowPlanExecutor *right_cardsflow, *left_cardsflow;
};


#endif //SRC_SCOOPING_H
