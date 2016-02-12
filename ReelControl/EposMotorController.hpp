// JDW 2016-2-10
// Copyright Creare 2016
#include <Definitions.h> // Comes with Epos_Linux_Library
#include <iostream>
#include <string>
#include <sstream>

class EposMotorController {
private:
    std::string portName;
    void * deviceHandle;
    unsigned int baudRate;

    std::string lookupError(unsigned int error);
    void enable(); // To avoid confusion, force them to clear the fault too
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
    
    
};
