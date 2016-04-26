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

    long ReelController::motor_position_from_tether_length(double tether_length_m) {
        return QC_PER_TURN * (tether_length_m / (M_PI * reel_diameter_m));
    }

    double ReelController::tether_length_from_motor_position(int motor_position) {
        return ((double)motor_position / QC_PER_TURN) * (M_PI * reel_diameter_m);
    }

    unsigned int ReelController::motor_rpm_from_tether_mps(double tether_mps) {
        return (60.0 * tether_mps / (M_PI * reel_diameter_m));
    }

    void ReelController::init() {
        if(motor_controller.isOpen()) {
            // Note: By convention, we will not apply settings to the motor controller here.
            // Instead, we will use the EPOS Studio to set up settings.
            // This is because we have a working set of settings, and because EPOS Studio provides
            // a more informative explanation of a lot of the settings, so it's harder to lose track
            // of, for example, which sensor you're configuring.
            int num = motor_controller.getGearRatioNumerator();
            int den = motor_controller.getGearRatioDenominator();
            gear_ratio = (double)num / (double)den;
            std::cout << "Got gear ratio - " << num << ":" << den << " = " << gear_ratio << std::endl;
            motor_controller.setOperatingMode(EposMotorController::EPOS_OPMODE_PROFILE_POSITION_MODE);
            motor_controller.clearFaultAndEnable();
        } else {
            std::cout << "Motor controller not open." << std::endl;
        }
    }

    void ReelController::setTetherToHome() {
        last_commanded_tether_length_m = 0;
    }

    void ReelController::setTetherLength(double desired_length_m) {
        // TBD: add offsets
        last_commanded_tether_length_m = desired_length_m;
        double desired_motor_position = motor_position_from_tether_length(desired_length_m);
        std::cout << "Moving motor to " << desired_motor_position << std::endl;
        motor_controller.moveToPosition((long)desired_motor_position);
    }

    double ReelController::setMaxTetherSpeed(double max_tether_mps)
    {
        unsigned int max_payout_rpm = motor_rpm_from_tether_mps(max_tether_mps);
        if(max_payout_rpm * gear_ratio > MOTOR_MAX_RPM)
        { std::cout << "Limiting max RPM to motor max" << std::endl; max_payout_rpm = MOTOR_MAX_RPM / gear_ratio; }
        if(max_payout_rpm * gear_ratio > GEARBOX_MAX_INPUT_RPM)
        { std::cout << "Limiting max RPM to gearbox max" << std::endl;  max_payout_rpm = GEARBOX_MAX_INPUT_RPM / gear_ratio; }
        std::cout << "Setting max payout velocity to " << max_tether_mps << "mps = " << max_payout_rpm << "RPM." << std::endl;
        motor_controller.setMaxVelocity(max_payout_rpm);
        return max_payout_rpm;
    }

    void ReelController::setTetherSpeed(double tether_mps)
    {
        unsigned int tether_rpm        = motor_rpm_from_tether_mps(tether_mps);
        unsigned int tether_accel_rpms = motor_rpm_from_tether_mps(profile_accel_mpss);
        unsigned int tether_decel_rpms = motor_rpm_from_tether_mps(profile_decel_mpss);
        std::cout << "Setting profile to /" << tether_accel_rpms
                  << " -" << tether_rpm << " \\" << tether_decel_rpms << std::endl;
        motor_controller.setPositionProfile(tether_rpm, tether_accel_rpms, tether_decel_rpms);
        if(!isnan(last_commanded_tether_length_m)) {
            setTetherLength(last_commanded_tether_length_m);
        }
    }

    void ReelController::setTetherAccelDecel(double accel_mpss, double decel_mpss)
    {
        // Store these for later
        profile_accel_mpss = accel_mpss;
        profile_decel_mpss = decel_mpss;
    }

    double ReelController::getTetherLength() {
        int cur_motor_position = motor_controller.getPosition();
        std::cout << "Motor position=" << cur_motor_position << " ";
        return tether_length_from_motor_position(cur_motor_position);
    }
}