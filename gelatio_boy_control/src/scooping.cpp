//
// Created by arne on 20.08.19.
//

#include "gelataio_boy_control/scooping.h"

ScoopingMain::ScoopingMain(ros::NodeHandle *handle) : left_arm("left", 5), right_arm("right", 5), node_handle(handle) {
    left_cardsflow = new CardsflowPlanExecutor("left", handle);
    left_arm.addPlanExecutor(left_cardsflow);

    right_cardsflow = new CardsflowPlanExecutor("right", handle);
    right_arm.addPlanExecutor(right_cardsflow);
}

ScoopingMain::~ScoopingMain() {

}

