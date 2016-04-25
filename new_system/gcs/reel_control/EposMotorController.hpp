// JDW 2016-2-10
// Copyright Creare 2016
#ifndef EPOS_MOTOR_CONTROLLER_H
#define EPOS_MOTOR_CONTROLLER_H

#include <Definitions.h> // Comes with Epos_Linux_Library
#include <iostream>
#include <string>
#include <sstream>

class EposMotorController {
public:
    enum OperatingMode {
        EPOS_OPMODE_UNKNOWN,
        EPOS_OPMODE_PROFILE_POSITION_MODE,
    };
private:
    std::string portName;
    void * deviceHandle;
    unsigned int baudRate;
    OperatingMode curOpMode;

    std::string lookupError(unsigned int error);
    void enable(); // To avoid confusion, force them to clear the fault too
    void failWithCode(std::string message, int error_code, bool disable_motor = true); // The method called when an error is detected
    void fail(std::string message, bool disable_motor = true); // The method called when an error is detected
    
    void haltPositionMovement(); // Users of this class should call haltMovement() instead
public:
    static const unsigned short NODE_ID; // This would matter much more in a CAN network
    static const unsigned short GEAR_RATIO_INDEX;// The Object Index of the Gear Configuration object in EPOS memory
    
    EposMotorController(std::string port = "USB0", unsigned int baudRate=1000000);
    ~EposMotorController();
    
    // Open/close the specified port. (throw an exception on failure)
    void open();
    void close();
    bool isOpen() { return deviceHandle != NULL; };

    // Enable/disable movement. (throw an exception on failure)
    void clearFaultAndEnable();
    void clearFault();
    void disable();
    bool isEnabled();
    bool isFaulted();
    
    // Configuration (throw an exception on failure)
    void setOperatingMode(OperatingMode mode);
    void setSensorType(unsigned short type); // Must be one of the "ST_..." constants from Definitions.h
    void setEncoderSettings(unsigned int pulses_per_turn=1024, bool invert_polarity=false);
    unsigned short getGearRatioNumerator();
    unsigned short getGearRatioDenominator();
    
    // Movement (throw an exception on failure)
    void moveToPosition(long position);
    int getPosition();
    void setMaxVelocity(unsigned int velocity); // velocity is in RPM after gearbox. Applies to both position and velocity control.
    void setPositionProfile(unsigned int velocity, unsigned int acceleration, unsigned int deceleration); // velocity/accel/decel is in RPM or RPM/s after gearbox.
    void haltMovement();
};

#endif // EPOS_MOTOR_CONTROLLER_H