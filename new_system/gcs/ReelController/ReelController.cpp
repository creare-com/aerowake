// JDW 2016-2-10
// Copyright Creare 2016
#include "ReelController.hpp"
namespace gcs {
    const unsigned int ReelController::QC_PER_TURN = 1024*4;
    const unsigned int ReelController::MOTOR_MAX_RPM = 10000;
    const unsigned int ReelController::GEARBOX_MAX_INPUT_RPM = 8000;

    ReelController::ReelController(std::string port, double reel_diam_cm) :
        motor_controller(port), reel_diameter_m(reel_diam_cm / 100.0), gear_ratio(26),
        profile_accel_mpss(0.6), profile_decel_mpss(0.6), last_commanded_tether_length_m(nan(""))
    {
        std::cout << "Initializing with port: " << port << " reel diameter:" 
            << reel_diameter_m << "m and qc/rev: " << QC_PER_TURN << std::endl;
        motor_controller.open();
        init();
    }

    long ReelController::motorPositionFromTetherLength(double tether_length_m) {
        return QC_PER_TURN * (-tether_length_m / (M_PI * reel_diameter_m));
    }

    double ReelController::tetherLengthFromMotorPosition(int motor_position) {
        return (-(double)motor_position / QC_PER_TURN) * (M_PI * reel_diameter_m);
    }

    unsigned int ReelController::motorRpmFromTetherMps(double tether_mps) {
        // Do not invert sign, since the motor controller only refers to rates in the positive
        return (60.0 * tether_mps / (M_PI * reel_diameter_m));
    }

    double ReelController::tetherMpsFromMotorRpm(unsigned int motor_rpm) {
        // Do not invert sign, since the motor controller only refers to rates in the positive
        return (double)motor_rpm * (M_PI * reel_diameter_m) / 60.0;
    }

    void ReelController::init() {
        if(motor_controller.isOpen()) {
            // Note: By convention, we will not apply settings to the motor controller here.
            // Instead, we will use the EPOS Studio to set up settings.
            // This is because we have a working set of settings, and because EPOS Studio provides
            // a more informative explanation of a lot of the settings, so it's harder to lose track
            // of, for example, which sensor you're configuring.
            
            // Get and remember gear ratio
            int num = motor_controller.getGearRatioNumerator();
            int den = motor_controller.getGearRatioDenominator();
            gear_ratio = (double)num / (double)den;
            std::cout << "Got gear ratio - " << num << ":" << den << " = " << gear_ratio << std::endl;
            
            // Get and remember acceleration/deceleration
            unsigned int tether_rpm       ;
            unsigned int tether_accel_rpms;
            unsigned int tether_decel_rpms;
            motor_controller.getPositionProfile(&tether_rpm, &tether_accel_rpms, &tether_decel_rpms);
            profile_accel_mpss = tetherMpsFromMotorRpm(tether_accel_rpms);
            profile_decel_mpss = tetherMpsFromMotorRpm(tether_decel_rpms);
                
            // Set up
            motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
            motor_controller.clearFaultAndEnable();
        } else {
            std::cout << "Motor controller not open." << std::endl;
        }
    }

    // Passthrough functions
    void ReelController::haltMovement() { motor_controller.haltMovement(); }
    void ReelController::disable() { motor_controller.disable(); }
    void ReelController::clearFaultAndEnable() { motor_controller.clearFaultAndEnable(); }
    bool ReelController::isEnabled() { return motor_controller.isEnabled(); }

    void ReelController::setTetherToHome() {
        last_commanded_tether_length_m = 0;
    }

    void ReelController::setTetherLength(double desired_length_m) {
        // TBD: add offsets
        last_commanded_tether_length_m = desired_length_m;
        double desired_motor_position = motorPositionFromTetherLength(desired_length_m);
        std::cout << "Moving motor to " << desired_motor_position << std::endl;
        motor_controller.moveToPosition((long)desired_motor_position);
    }

    double ReelController::setMaxTetherSpeed(double max_tether_mps) {
        unsigned int max_payout_rpm = motorRpmFromTetherMps(max_tether_mps);
        if(max_payout_rpm * gear_ratio > MOTOR_MAX_RPM)
        { std::cout << "Limiting max RPM to motor max" << std::endl; max_payout_rpm = MOTOR_MAX_RPM / gear_ratio; }
        if(max_payout_rpm * gear_ratio > GEARBOX_MAX_INPUT_RPM)
        { std::cout << "Limiting max RPM to gearbox max" << std::endl;  max_payout_rpm = GEARBOX_MAX_INPUT_RPM / gear_ratio; }
        std::cout << "Setting max payout velocity to " << max_tether_mps << "mps = " << max_payout_rpm << "RPM." << std::endl;
        motor_controller.setMaxVelocity(max_payout_rpm);
        return max_payout_rpm;
    }

    double ReelController::getMaxTetherSpeed() {
        double max_payout_rpm = motor_controller.getMaxVelocity();
        return tetherMpsFromMotorRpm(max_payout_rpm);
    }

    void ReelController::setTetherSpeed(double tether_mps) {
        unsigned int tether_rpm        = motorRpmFromTetherMps(fabs(tether_mps));
        unsigned int tether_accel_rpms = motorRpmFromTetherMps(profile_accel_mpss);
        unsigned int tether_decel_rpms = motorRpmFromTetherMps(profile_decel_mpss);
        std::cout << "Setting profile to /" << tether_accel_rpms
                  << " " << tether_rpm << " \\" << tether_decel_rpms << std::endl;
        motor_controller.setPositionProfile(tether_rpm, tether_accel_rpms, tether_decel_rpms);
        if(!isnan(last_commanded_tether_length_m)) {
            setTetherLength(last_commanded_tether_length_m);
        }
    }

    double ReelController::getTetherSpeed() {
        unsigned int tether_rpm       ;
        unsigned int tether_accel_rpms;
        unsigned int tether_decel_rpms;
        motor_controller.getPositionProfile(&tether_rpm, &tether_accel_rpms, &tether_decel_rpms);
        
        return tetherMpsFromMotorRpm(tether_rpm);
    }

    void ReelController::setTetherAccelDecel(double accel_mpss, double decel_mpss)
    {
        // Store these for later; we'll send them every time we set the speed
        profile_accel_mpss = fabs(accel_mpss);
        profile_decel_mpss = fabs(decel_mpss);
    }

    double ReelController::getTetherLength() {
        int cur_motor_position = motor_controller.getPosition();
        std::cout << "Motor position=" << cur_motor_position << " ";
        return tetherLengthFromMotorPosition(cur_motor_position);
    }
}