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
    ~EposMotorController();
    
    // All the following functions throw an exception on failure.
    
    // Open/close the specified port.
    void open();
    void close();
    
    // EPOS state change functions - see page 3-14 of the firmware manual
    void sendShutdownCmd();
    void sendSwitchOnCmd();
    void sendDisableOpsCmd();
    void sendEnableOpsCmd();
    void sendQuickstopCmd();
    void sendDisableVoltageCmd();
    void sendFaultResetCmd();
    
    // Note that these states are from pg 3-14
    // of the firmware manual, not the simplified ones from
    // Definitions.h.
    enum State {
        START,
        NOT_READY_TO_SWITCH_ON,
        SWITCH_ON__DISABLED,
        READY_TO_SWITCH_ON,
        SWITCHED_ON,
        REFRESH,
        MEASURE_INIT,
        OPERATION_ENABLE,
        QUICKSTOP_ACTIVE,
        FAULT,
        FAULT_REACTION_ACTIVE_DIS,
        FAULT_REACTION_ACTIVE_EN,
    };
    State getState();
};

std::string lookupError(unsigned int error);