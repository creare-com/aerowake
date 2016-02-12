// JDW 2016-2-10
// Copyright Creare 2016
#ifndef REEL_CONTROLLER_H
#define REEL_CONTROLLER_H

#include <iostream>
#include <string>
#include "EposMotorController.hpp"

class ReelController {
private:
    EposMotorController motor_controller;
    void init();
public:
    ReelController(std::string port = "/dev/ttyS0");
};

#endif // REEL_CONTROLLER_H