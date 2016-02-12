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
    
    std::cout << "Configuring motor controller...";
    try {
        init();
        std::cout << " done." << std::endl;
    } catch (std::exception& e) {
        std::cout << std::endl << "Got exception while configuring motor controller: "
            << e.what() << std::endl;
    }
    
    long pos = 30;
    std::cout << "Setting position to " << pos << "... " << std::endl;
    try {
        motor_controller.moveToPosition(pos);
        std::cout << " done." << std::endl;
    } catch (std::exception& e) {
        std::cout << std::endl << "Got exception while commanding motor controller: "
            << e.what() << std::endl;
    }
    
    int sleep_time = 10;
    std::cout << "Waiting " << sleep_time << " seconds." << std::endl;
    sleep(sleep_time);
    std::cout << "Exiting." << std::endl;
}
void ReelController::init() {
    if(motor_controller.isOpen()) {
        motor_controller.setMaxVelocity(50);
        motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
        motor_controller.clearFaultAndEnable();
    }
}