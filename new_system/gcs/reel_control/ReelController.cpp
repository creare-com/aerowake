// JDW 2016-2-10
// Copyright Creare 2016
#include "ReelController.hpp"
const unsigned int ReelController::QC_PER_TURN = 1024*4;

ReelController::ReelController(std::string port, double reel_diam_cm) :
    motor_controller(port), reel_diameter_m(reel_diam_cm / 100.0), 
    max_payout_velocity_mps(0.10)
{
    motor_controller.open();
    init();
}

long ReelController::motor_position_from_tether_length(double tether_length_m) {
    return QC_PER_TURN * (tether_length_m / (M_PI * reel_diameter_m));
    // return tether_length_m * 10270.0; // Anecdotal pulses/meter
}

double ReelController::tether_length_from_motor_position(int motor_position) {
    return ((double)motor_position / QC_PER_TURN) * (M_PI * reel_diameter_m);
    // return motor_position / 10270.0; // Anecdotal pulses/meter
}

unsigned int ReelController::motor_rpm_from_tether_mps(double tether_mps) {
    return tether_mps / (M_PI * reel_diameter_m);
}

void ReelController::init() {
    if(motor_controller.isOpen()) {
        // Note: By convention, we will not apply settings to the motor controller here.
        // Instead, we will use the EPOS Studio to set up settings.
        // This is because we have a working set of settings, and because EPOS Studio provides
        // a more informative explanation of a lot of the settings, so it's harder to lose track
        // of, for example, which sensor you're configuring.
        
        // motor_controller.setMaxVelocity(1);
        motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
        motor_controller.clearFaultAndEnable();
    } else {
        std::cout << "Motor controller not open." << std::endl;
    }
}

void ReelController::test() {
    double len = getTetherLength();
    std::cout << "Tether length=" << len << std::endl;
    len += 0.1;
    std::cout << "Motor controller is " << (motor_controller.isEnabled()? "" : "not ") << "enabled." << std::endl;
    std::cout << "Motor controller is " << (motor_controller.isFaulted()? "" : "not ") << "faulted." << std::endl;
    std::cout << "Setting length to " << len << std::endl;
    setTetherLength(len);
    std::cout << "Motor controller is " << (motor_controller.isEnabled()? "" : "not ") << "enabled." << std::endl;
    std::cout << "Motor controller is " << (motor_controller.isFaulted()? "" : "not ") << "faulted." << std::endl;
    // std::cout << "Disabling..." << std::endl;
    // motor_controller.disable();
    // std::cout << "Motor controller is " << (motor_controller.isEnabled()? "" : "not ") << "enabled." << std::endl;
    // std::cout << "Motor controller is " << (motor_controller.isFaulted()? "" : "not ") << "faulted." << std::endl;
}

void ReelController::setTetherLength(double desired_length_m) {
    // TBD: add offsets
    double desired_motor_position = motor_position_from_tether_length(desired_length_m);
    std::cout << "Moving motor to " << desired_motor_position << std::endl;
    motor_controller.moveToPosition((long)desired_motor_position);
}

double ReelController::getTetherLength() {
    int cur_motor_position = motor_controller.getPosition();
    std::cout << "Motor position=" << cur_motor_position << " ";
    return tether_length_from_motor_position(cur_motor_position);
}
