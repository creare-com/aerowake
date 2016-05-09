// JDW 2016-2-10
// Copyright Creare 2016
#ifndef REEL_CONTROLLER_H
#define REEL_CONTROLLER_H

#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>
#include "EposMotorController.hpp"
namespace gcs {
    class ReelController {
    private:
        EposMotorController motor_controller;
        double reel_diameter_m; // The diameter of the part that the line wraps around.
        double gear_ratio; // Ratio here is motor:output - for instance, 26:1 = 26 would indicate 26 turns on the motor corresponding to 1 turn on the reel.
        
        long motorPositionFromTetherLength(double tether_length_m); // in pulses
        double tetherLengthFromMotorPosition(int motor_position); // in meters
        unsigned int motorRpmFromTetherMps(double tether_mps);
        double tetherMpsFromMotorRpm(unsigned int motor_rpm);
        void init();
        
    public:
        static const unsigned int QC_PER_TURN;
        static const unsigned int MOTOR_MAX_RPM;
        static const unsigned int GEARBOX_MAX_INPUT_RPM;
        ReelController(std::string port = "USB0", double reel_diam_cm=12.7); // 12.7cm = 5"

        // Basic motor operations
        void haltMovement(); // Cancel last movement - tries to hold the tether at this length
        void disable(); // permit the tether to spool freely
        void clearFaultAndEnable(); // enable movement control; hold tether at this length
        bool isEnabled(); // returns true if the motor controller is attempting to hold position
        
        // Tether operations
        void setTetherToHome(); // consider the tether length to be 0; assume we're fully reeled in.
        void setTetherLength(double desired_length_m); // pays out or reels in the tether to this length
        double setMaxTetherSpeed(double max_tether_mps); // Returns the actual max payout/retract rate set.  Will cap based on the motor & gearbox capabilities.
        double getMaxTetherSpeed(); // Returns the actual max payout/retract rate.
        double getTetherLength();
        double getTetherTargetLength();
    };
}

#endif // REEL_CONTROLLER_H

