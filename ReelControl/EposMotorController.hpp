// JDW 2016-2-10
// Copyright Creare 2016
#ifndef EPOS_MOTOR_CONTROLLER_H
#define EPOS_MOTOR_CONTROLLER_H

#include <Definitions.h> // Comes with Epos_Linux_Library
#include <iostream>
#include <string>
#include <sstream>

class EposMotorController {
private:
    std::string portName;
    void * deviceHandle;
    unsigned int baudRate;
    enum OperatingMode {
        EPOS_OPMODE_UNKNOWN,
        EPOS_OPMODE_PROFILE_POSITION_MODE,
    };
    OperatingMode curOpMode;

    std::string lookupError(unsigned int error);
    void enable(); // To avoid confusion, force them to clear the fault too
    void fail(std::string message, int error_code, bool disable_motor); // The method called when an error is detected
    void fail(std::string message, bool disable_motor); // The method called when an error is detected
    
    void setOperatingMode(OperatingMode mode);
    void haltPositionMovement();
public:
    static const unsigned short NODE_ID; // This would matter much more in a CAN network
    
    EposMotorController(std::string port = "/dev/ttyS0", unsigned int baudRate=1000000);
    ~EposMotorController();
    
    // Open/close the specified port. (throw an exception on failure)
    void open();
    void close();

    // Enable/disable movement. (throw an exception on failure)
    void clearFaultAndEnable();
    void clearFault();
    void disable();
    bool isEnabled();
    bool isFaulted();
    
    // Command the motor's movement
    void moveToPosition(long position);
    void haltMovement();
};

#endif // EPOS_MOTOR_CONTROLLER_H