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
public:
    EposMotorController(std::string port = "/dev/ttyS0", unsigned int baudRate=1000000);
    
    // Open the specified port.  Throws an exception on failure.
    void open();
};

std::string lookupError(unsigned int error);