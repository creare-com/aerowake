// JDW 2016-2-10
// Copyright Creare 2016
#include "EposMotorController.hpp"
const unsigned short EposMotorController::NODE_ID = 1;// This would matter much more in a CAN network

EposMotorController::EposMotorController(std::string port, unsigned int baudRate) :
    portName(port), baudRate(baudRate), curOpMode(EPOS_OPMODE_UNKNOWN)
{

}

EposMotorController::~EposMotorController() {
    close(); // Also disables
}

void EposMotorController::open() 
{
    unsigned int error_code = 0;
    
    deviceHandle = VCS_OpenDevice (
        (char*)"EPOS2",
        (char*)"MAXON SERIAL V2",
        (char*)"USB",
        (char*)portName.c_str(),
        &error_code);
        
    if(deviceHandle != 0 && error_code == 0) {
        // This get-set-get pattern lets us change just the baud rate,
        // then confirm it took.
        unsigned int actual_baud_rate = 0;
        unsigned int actual_timeout = 0;
        if(0 != VCS_GetProtocolStackSettings(deviceHandle, &actual_baud_rate, &actual_timeout, &error_code)) {
            if(0 != VCS_SetProtocolStackSettings(deviceHandle, baudRate, actual_timeout, &error_code)) {
                if(0 != VCS_GetProtocolStackSettings(deviceHandle, &actual_baud_rate, &actual_timeout, &error_code)) {
                    if (actual_baud_rate == baudRate) {
                        // Right now we only support one operating mode; enter it
                        setOperatingMode(EPOS_OPMODE_PROFILE_POSITION_MODE);
                    } else {
                        std::stringstream ss;
                        ss << "Tried to set port rate to " << baudRate << " but instead got " << actual_baud_rate << std::endl;
                        fail(ss.str(), error_code);
                    }
                } else { fail("Failed to confirm port settings", error_code, true); }
            } else { fail("Failed to write port settings", error_code, true); }
        } else { fail("Failed to read port settings", error_code, true); }
    } else { fail("Failed to open motor controller", error_code, true); }
}
/************************************
             Open/close
************************************/
void EposMotorController::close() {
    disable(); // turn it off before releasing control
    setOperatingMode(EPOS_OPMODE_UNKNOWN);
    unsigned int error_code = 0;
    if(VCS_CloseDevice(deviceHandle, &error_code) == 0)
    { fail("Failed to close motor controller", error_code, false); }
}
void EposMotorController::clearFaultAndEnable() {
    if (isFaulted()) { clearFault(); }
    enable();
}

/************************************
             State
************************************/
void EposMotorController::clearFault() {
    unsigned int error_code = 0;
    if(VCS_ClearFault(deviceHandle, NODE_ID, &error_code) == 0)
    { fail("Failed to clear fault", error_code, true); }
}

void EposMotorController::enable() {
    unsigned int error_code = 0;
    if(VCS_SetEnableState(deviceHandle, NODE_ID, &error_code) == 0)
    { fail("Failed to set enabled state", error_code, true); }
}

void EposMotorController::disable() {
    unsigned int error_code = 0;
    if(VCS_SetDisableState(deviceHandle, NODE_ID, &error_code) == 0)
    { fail("Failed to set disabled state", error_code, false); }
}

bool EposMotorController::isEnabled() {
    unsigned int error_code = 0;
    int enabled = 0;
    if(VCS_GetEnableState(deviceHandle, NODE_ID, &enabled, &error_code) != 0)
    {
        if(enabled) { return true; }
        else        { return false; }    
    } else { fail("Failed to read fault state", error_code, true); return false; }
}

bool EposMotorController::isFaulted() {
    unsigned int error_code = 0;
    int faulted = 0;
    if(VCS_GetFaultState(deviceHandle, NODE_ID, &faulted, &error_code) != 0)
    {
        if(faulted) { return true; }
        else        { return false; }
    } else { fail("Failed to read fault state", error_code, true); return false; }
}

/************************************
             Configuration
************************************/
void EposMotorController::setOperatingMode(OperatingMode mode) {
    unsigned int error_code = 0;
    switch(mode) {
        case EPOS_OPMODE_PROFILE_POSITION_MODE:
            if(VCS_ActivateProfilePositionMode(deviceHandle, NODE_ID, &error_code) == 0)
            { fail("Failed to activate Profile Positioning Mode", error_code, true); }
            break;
        case EPOS_OPMODE_UNKNOWN:
        default:
            // Handled at the end of this function
            break;
    }
    
    curOpMode = mode;
}

void EposMotorController::setSensorType(unsigned short type) {
    switch (type) {
        case ST_UNKNOWN               :
        case ST_INC_ENCODER_3CHANNEL  :
        case ST_INC_ENCODER_2CHANNEL  :
        case ST_HALL_SENSORS          :
        case ST_SSI_ABS_ENCODER_BINARY:
        case ST_SSI_ABS_ENCODER_GREY  :
        {
            unsigned int error_code = 0;
            if(VCS_SetSensorType(deviceHandle, NODE_ID, type, &error_code) == 0)
            { fail("Failed to set sensor type", error_code, true); }
            break;
        }
        default:
        {
            std::stringstream ss; 
            ss << "Cannot set sensor type " << type << "; not a valid type.";
            fail(ss.str(), true);
            break;
        }
    }
}

void EposMotorController::setEncoderSettings(unsigned int pulses_per_turn, bool invert_polarity) {
    unsigned int error_code = 0;
    if(VCS_SetIncEncoderParameter(deviceHandle, NODE_ID, pulses_per_turn, invert_polarity, &error_code) == 0)
    { fail("Failed to set encoder parameters", error_code, false); }
}

/************************************
             Movement
************************************/
void EposMotorController::moveToPosition(long position) {
    if(curOpMode == EPOS_OPMODE_PROFILE_POSITION_MODE) {
        unsigned int error_code = 0;
        if(VCS_MoveToPosition(deviceHandle,
            NODE_ID, 
            position, 
            1, // absolute = TRUE
            1, // cancel the last one = TRUE
            &error_code) == 0)
        { fail("Failed to command movement to position", error_code, true); }
    } else {
        std::stringstream ss;
        ss << "Cannot move to a position unless in a positioning mode.  Currently in mode " << curOpMode << ".";
        fail(ss.str(), true);
    }
}

void EposMotorController::setMaxVelocity(unsigned int velocity) {
    unsigned int error_code = 0;
    if(VCS_SetMaxProfileVelocity(deviceHandle, NODE_ID, velocity, &error_code) == 0)
    { fail("Failed to set maximum velocity", error_code, true); }
}

void EposMotorController::haltMovement() {
    switch(curOpMode) {
        case EPOS_OPMODE_PROFILE_POSITION_MODE:
        default:
            haltPositionMovement();
            break;
    }
}

void EposMotorController::haltPositionMovement() {
    unsigned int error_code = 0;
    if(VCS_HaltPositionMovement(deviceHandle, NODE_ID, &error_code) == 0)
    { fail("Failed to halt position movement", error_code, false); }
}

/************************************
           Error handling
************************************/

void EposMotorController::fail(std::string message, bool disable_motor) {
    std::cout << message << std::endl;
    if(disable_motor) {
        disable(); // Note that this is likely to throw an exception, so code after this line may not execute.
    }
    throw std::exception();
}

void EposMotorController::fail(std::string message, int error_code, bool disable_motor) {
    std::stringstream ss;
    ss << message << ": " << lookupError(error_code);
    fail(ss.str(), disable_motor);
}

std::string EposMotorController::lookupError(unsigned int error) {
    switch(error) {
        case 0x00000000: return "Function was successful"; break;
        case 0x05030000: return "Toggle bit not alternated"; break;
        case 0x05040000: return "SDO protocol timed out"; break;
        case 0x05040001: return "Client/server command specifier not valid or unknown"; break;
        case 0x05040002: return "Invalid block size (block mode only)"; break;
        case 0x05040003: return "Invalid sequence number (block mode only)"; break;
        case 0x05040004: return "CRC error (block mode only)"; break;
        case 0x05040005: return "Out of Memory"; break;
        case 0x06010000: return "Unsupported access to an object (e.g. write command to a read-only object)"; break;
        case 0x06010001: return "Read command to a write only object"; break;
        case 0x06010002: return "Write command to a read only object"; break;
        case 0x06020000: return "Last read or write command had a wrong object index or subindex"; break;
        case 0x06040041: return "Object cannot be mapped to PDO"; break;
        case 0x06040042: return "Number and length of objects to be mapped would exceed PDOlength"; break;
        case 0x06040043: return "General parameter incompatibility"; break;
        case 0x06040047: return "General internal incompatibility in device"; break;
        case 0x06060000: return "Access failed due to an hardware error"; break;
        case 0x06070010: return "Data type does not match, length or service parameter does notmatch"; break;
        case 0x06070012: return "Data type does not match, length or service parameter too high"; break;
        case 0x06070013: return "Data type does not match, length or service parameter too low"; break;
        case 0x06090011: return "Last read or write command had a wrong subindex"; break;
        case 0x06090030: return "Value range of parameter exceeded"; break;
        case 0x06090031: return "Value of parameter written too high"; break;
        case 0x06090032: return "Value of parameter written too low"; break;
        case 0x06090036: return "Maximum value is less than minimum value"; break;
        case 0x08000000: return "General error"; break;
        case 0x08000020: return "Data cannot be transferred or stored"; break;
        case 0x08000021: return "Data cannot be transferred or stored to application because of local control"; break;
        case 0x08000022: return "Data cannot be transferred or stored to application because of present device state"; break;
        case 0x0F00FFB9: return "Wrong CAN id"; break;
        case 0x0F00FFBC: return "Device is not in service mode"; break;
        case 0x0F00FFBE: return "Password is wrong"; break;
        case 0x0F00FFBF: return "RS232 command is illegal (does not exist)"; break;
        case 0x0F00FFC0: return "Device is in wrong NMT state"; break;
        case 0x10000001: return "Internal error"; break;
        case 0x10000002: return "Null pointer passed to function"; break;
        case 0x10000003: return "Handle passed to function is not valid"; break;
        case 0x10000004: return "Virtual device name is not valid"; break;
        case 0x10000005: return "Device name is not valid"; break;
        case 0x10000006: return "Protocol stack name is not valid"; break;
        case 0x10000007: return "Interface name is not valid"; break;
        case 0x10000008: return "Port is not valid"; break;
        case 0x10000009: return "Could not load external library"; break;
        case 0x1000000A: return "Command failed"; break;
        case 0x1000000B: return "Timeout occurred during execution"; break;
        case 0x1000000C: return "Bad parameter passed to function"; break;
        case 0x1000000D: return "Command aborted by user"; break;
        case 0x1000000E: return "Buffer is too small"; break;
        case 0x1000000F: return "No communication settings found"; break;
        case 0x10000010: return "Function not supported"; break;
        case 0x10000011: return "Parameter already used"; break;
        case 0x10000020: return "Bad device state"; break;
        case 0x10000021: return "Bad file content"; break;
        case 0x10000022: return "System cannot find specified path"; break;
        case 0x20000001: return "Error opening interface"; break;
        case 0x20000002: return "Error closing interface"; break;
        case 0x20000003: return "Interface is not open"; break;
        case 0x20000004: return "Error opening port"; break;
        case 0x20000005: return "Error closing port"; break;
        case 0x20000006: return "Port is not open"; break;
        case 0x20000007: return "Error resetting port"; break;
        case 0x20000008: return "Error configuring port settings"; break;
        case 0x20000009: return "Error configuring port mode"; break;
        case 0x23000001: return "Error writing USB data"; break;
        case 0x23000002: return "Error reading USB data"; break;
        case 0x34000001: return "Failed stuffing USB data"; break;
        case 0x34000002: return "Failed destuffing USB data"; break;
        case 0x34000003: return "Bad USB CRC received"; break;
        case 0x34000004: return "Bad USB data received"; break;
        case 0x34000005: return "Bad USB data size written"; break;
        case 0x34000006: return "Failed writing USB data"; break;
        case 0x34000007: return "Failed reading USB data"; break;
        default:
        {
            std::stringstream ss;
            ss << "Unknown error: 0x" << std::hex << error;
            return ss.str();
        }
    }
}
