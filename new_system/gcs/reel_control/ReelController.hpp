// JDW 2016-2-10
// Copyright Creare 2016
#ifndef REEL_CONTROLLER_H
#define REEL_CONTROLLER_H

#include <iostream>
#include <string>
#include <math.h>
#include "EposMotorController.hpp"

class ReelController {
private:
    EposMotorController motor_controller;
    double reel_diameter_m;
    double max_payout_velocity_mps; // meters per second

    long motor_position_from_tether_length(double tether_length_m); // in pulses
    double tether_length_from_motor_position(int motor_position); // in meters
    unsigned int motor_rpm_from_tether_mps(double tether_mps);
    void init();
public:
    static const unsigned int PULSES_PER_TURN;
    ReelController(std::string port = "USB0", double reel_diam_cm=10.0);
    void test();
    
    // Tether operations
    void tetherIsHome(); // consider the tether length to be 0, we're fully reeled in.
    void setTetherLength(double desired_length_m); // pays out or reels in the tether to this length
    double getTetherLength();
};

#endif // REEL_CONTROLLER_H
