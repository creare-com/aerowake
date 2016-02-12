// JDW 2016-2-10
// Copyright Creare 2016
#include "ReelController.hpp"

ReelController::ReelController(std::string port) :
    motor_controller(port) 
{
    motor_controller.open();
    init();
}
void ReelController::init() {
    if(motor_controller.isOpen()) {
        motor_controller.setSensorType(ST_INC_ENCODER_3CHANNEL);
        motor_controller.setEncoderSettings(500, false);
        motor_controller.setMaxVelocity(50);
        motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
        motor_controller.clearFaultAndEnable();
    }
}

void ReelController::test() {
    long pos = 30;
    std::cout << "Setting position to " << pos << "... " << std::endl;
    motor_controller.moveToPosition(pos);

}
