// JDW 2016-2-10
// Copyright Creare 2016
#include "ReelController.hpp"
const unsigned int ReelController::PULSES_PER_TURN = 500;

ReelController::ReelController(std::string port, double reel_diam_cm) :
    motor_controller(port), reel_diameter_m(reel_diam_cm / 100.0), 
    max_payout_velocity_mps(0.10)
{
    motor_controller.open();
    init();
}

long ReelController::motor_position_from_tether_length(double tether_length_m) {
    return PULSES_PER_TURN * (tether_length_m / (M_PI * reel_diameter_m));
}

unsigned int ReelController::motor_rpm_from_tether_mps(double tether_mps) {
    return tether_mps / (M_PI * reel_diameter_m);
}

void ReelController::init() {
    if(motor_controller.isOpen()) {
        motor_controller.setSensorType(ST_INC_ENCODER_3CHANNEL);
        motor_controller.setEncoderSettings(PULSES_PER_TURN, false);
        motor_controller.setMaxVelocity(
            motor_rpm_from_tether_mps (max_payout_velocity_mps));
        motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
        motor_controller.clearFaultAndEnable();
    }
}

void ReelController::test() {
    double len_m = 1.0;
    std::cout << "Setting tether length to " << len_m << "m = " << 
        motor_position_from_tether_length(len_m) << "ticks." << std::endl;
    setTetherLength(1.0);
}

void ReelController::tetherIsHome() {
    // TBD.  Need to see how the motor controller handles homing.
    // In all likelihood, there will be some offset we track, between
    // the motor's perception of its position (which, mind you, is in ticks)
    // and the actual length of the tether.  There will be offset due to
    // how much tether was paid out when the motor controller homed, offset
    // due to clutch slippage, and offset due to the difference between the
    // reel attachment point and the encoder index point.
}

void ReelController::setTetherLength(double desired_length_m) {
    // TBD: add offsets
    double desired_motor_position = motor_position_from_tether_length(desired_length_m);
    motor_controller.moveToPosition((long)desired_motor_position);
}
