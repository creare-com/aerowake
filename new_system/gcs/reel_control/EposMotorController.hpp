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
    void fail(std::string message, int error_code, bool disable_motor); // The method called when an error is detected
    void fail(std::string message, bool disable_motor); // The method called when an error is detected
    
    void haltPositionMovement(); // Users of this class should call haltMovement() instead
public:
    static const unsigned short NODE_ID; // This would matter much more in a CAN network
    
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
    
    // Movement (throw an exception on failure)
    void moveToPosition(long position);
    int getPosition();
    void setMaxVelocity(unsigned int velocity); // Applies to both position and velocity control
    void haltMovement();
};

#endif // EPOS_MOTOR_CONTROLLER_H