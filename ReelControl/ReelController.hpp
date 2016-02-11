// JDW 2016-2-10
// Copyright Creare 2016
#include <iostream>
#include <string>
#include "EposMotorController.hpp"

class ReelController {
private:
    EposMotorController motor_controller;
public:
    ReelController(std::string port = "/dev/ttyS0");
};
