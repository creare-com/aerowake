// JDW 2016-2-10
// Copyright Creare 2016
#include "ReelController.hpp"

ReelController::ReelController(std::string port) :
    motor_controller(port) 
{
    std::cout << "Opening motor controller...";
    try {
        motor_controller.open();
        std::cout << " done." << std::endl;
    } catch (std::exception& e) {
        std::cout << std::endl << "Got exception while opening motor controller: "
            << e.what() << std::endl;
    }
}
