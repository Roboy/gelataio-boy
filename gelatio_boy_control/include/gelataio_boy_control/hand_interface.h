//
// Created by arne on 16.08.19.
//

#ifndef SRC_HAND_INTERFACE_H
#define SRC_HAND_INTERFACE_H


#include <string>

class hand_interface {
public:
    virtual bool grasp() = 0;

    virtual bool release() = 0;
};

class DummyHand : public hand_interface {
public:
    DummyHand(std::string hand_name) : hand_name(hand_name) {}

    bool grasp() override;

    bool release() override;

private:
    std::string hand_name;
};


#endif //SRC_HAND_INTERFACE_H
