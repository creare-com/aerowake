// JDW 2016-2-10
// Copyright Creare 2016
#include "EposMotorController.hpp"
namespace gcs {
    const unsigned short EposMotorController::NODE_ID = 1;// This would matter much more in a CAN network
    const unsigned short EposMotorController::GEAR_RATIO_INDEX = 0x2230;// The Object Index of the Gear Configuration object in EPOS memory

    EposMotorController::EposMotorController(std::string port, unsigned int baudRate) :
        portName(port), baudRate(baudRate), curOpMode(0)
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
                            // Default to a mode
                            setOperatingMode(OMD_PROFILE_POSITION_MODE);
                        } else {
                            std::stringstream ss;
                            ss << "Tried to set port rate to " << baudRate << " but instead got " << actual_baud_rate << std::endl;
                            failWithCode(ss.str(), error_code);
                        }
                    } else { failWithCode("Failed to confirm port settings", error_code); }
                } else { failWithCode("Failed to write port settings", error_code); }
            } else { failWithCode("Failed to read port settings", error_code); }
        } else { failWithCode("Failed to open motor controller", error_code); }
    }
    /************************************
                 Open/close
    ************************************/
    void EposMotorController::close() {
        disable(); // turn it off before releasing control
        
        unsigned int error_code = 0;
        if(VCS_CloseDevice(deviceHandle, &error_code) == 0)
        { failWithCode("Failed to close motor controller", error_code, false); }
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
        { failWithCode("Failed to clear fault", error_code); }
    }

    void EposMotorController::enable() {
        unsigned int error_code = 0;
        if(VCS_SetEnableState(deviceHandle, NODE_ID, &error_code) == 0)
        { failWithCode("Failed to set enabled state", error_code); }
    }

    void EposMotorController::disable() {
        unsigned int error_code = 0;
        if(VCS_SetDisableState(deviceHandle, NODE_ID, &error_code) == 0)
        { failWithCode("Failed to set disabled state", error_code, false); }
    }

    bool EposMotorController::isEnabled() {
        unsigned int error_code = 0;
        int enabled = 0;
        if(VCS_GetEnableState(deviceHandle, NODE_ID, &enabled, &error_code) != 0)
        {
            if(enabled) { return true; }
            else        { return false; }    
        } else { failWithCode("Failed to read fault state", error_code); return false; }
    }

    bool EposMotorController::isFaulted() {
        unsigned int error_code = 0;
        int faulted = 0;
        if(VCS_GetFaultState(deviceHandle, NODE_ID, &faulted, &error_code) != 0)
        {
            if(faulted) { return true; }
            else        { return false; }
        } else { failWithCode("Failed to read fault state", error_code); return false; }
    }
    
    
    /**
     * Gets the number of device errors.  These describe errors (if any)
     * within the motor controller device, rather than errors in the communications
     * library.  Communications library errors are thrown as std::runtime_error.
     * This is a prerequisite for calling getDeviceError() correctly.
     */
    unsigned char EposMotorController::getDeviceErrorCount() {
        unsigned int error_code = 0; // Communications library error code
        unsigned char device_error_count = 0;
        if(VCS_GetNbOfDeviceError(deviceHandle, NODE_ID, &device_error_count, &error_code) != 0)
        {
            return device_error_count;
        } else { failWithCode("Failed to read controller error count", error_code); return 0; }
    }

    /**
     * Gets a device error string.  These describe errors (if any)
     * within the motor controller device, rather than errors in the communications
     * library.  Communications library errors are thrown as std::runtime_error.
     * The device can report multiple errors, so you should call
     * getDeviceErrorCount() before calling this function.
     * You must call this function with 0 <= error_num < getDeviceErrorCount().
     */
    std::string EposMotorController::getDeviceError(unsigned char device_error_num) {
        unsigned int error_code = 0; // Communications library error code
        unsigned int device_error_code = 0;
        if(VCS_GetDeviceErrorCode(deviceHandle, NODE_ID, device_error_num, &device_error_code, &error_code) != 0)
        {
            return lookupDeviceError(device_error_code);
        } else { failWithCode("Failed to read controller error", error_code); return ""; }
    }
    
    /************************************
                 Configuration
    ************************************/
    /**
     * Reads an "object" from the "object dictionary", which is used to configure the motor
     * controller.  Will write up to bytes_to_read bytes to *out_data.
     *
     *
     */
    void EposMotorController::getObject(unsigned short obj_idx, unsigned char obj_sub_idx, void * out_data, unsigned int bytes_to_read) {
        unsigned int error_code = 0;
        unsigned int bytes_read = 0;
        if(VCS_GetObject(deviceHandle, NODE_ID, obj_idx, obj_sub_idx, out_data, bytes_to_read, &bytes_read, &error_code) != 0)
        {
            // No further action required
        } else { failWithCode("Failed to read object", error_code); }
    }
    void EposMotorController::setObject(unsigned short obj_idx, unsigned char obj_sub_idx, void * in_data, unsigned int bytes_to_write) {
        unsigned int error_code = 0;
        unsigned int bytes_written = 0;
        if(VCS_SetObject(deviceHandle, NODE_ID, obj_idx, obj_sub_idx, in_data, bytes_to_write, &bytes_written, &error_code) != 0)
        {
            // No further action required
        } else { failWithCode("Failed to write object", error_code); }
    }
    /**
     * Writes the object dictionary to nonvolatile memory
     *
     */
    void EposMotorController::storeAllObjects() {
        unsigned int error_code = 0;
        if(VCS_Store(deviceHandle, NODE_ID, &error_code) != 0)
        {
            // No further action required
        } else { failWithCode("Failed to store object dictionary to nonvolatile memory", error_code); }
    }
    void EposMotorController::setOperatingMode(signed char mode) {
        unsigned int error_code = 0;
        switch(mode) {
            case OMD_PROFILE_POSITION_MODE:
                if(VCS_ActivateProfilePositionMode(deviceHandle, NODE_ID, &error_code) == 0)
                { failWithCode("Failed to activate Profile Position Mode", error_code); }
                break;
            case OMD_PROFILE_VELOCITY_MODE:
                if(VCS_ActivateProfileVelocityMode(deviceHandle, NODE_ID, &error_code) == 0)
                { failWithCode("Failed to activate Profile Velocity Mode", error_code); }
                break;
            default:
            {
                std::stringstream ss;
                ss << "Operating mode " << mode << " not supported.";
                fail(ss.str());
                break;
            }
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
                { failWithCode("Failed to set sensor type", error_code); }
                break;
            }
            default:
            {
                std::stringstream ss; 
                ss << "Cannot set sensor type " << type << "; not a valid type.";
                fail(ss.str());
                break;
            }
        }
    }

    void EposMotorController::setEncoderSettings(unsigned int pulses_per_turn, bool invert_polarity) {
        unsigned int error_code = 0;
        if(VCS_SetIncEncoderParameter(deviceHandle, NODE_ID, pulses_per_turn, invert_polarity, &error_code) == 0)
        { failWithCode("Failed to set encoder parameters", error_code); }
    }

    unsigned short EposMotorController::getGearRatioNumerator()
    {
        unsigned int error_code = 0;
        unsigned char NUMERATOR_SUB_INDEX = 0x01;
        unsigned short numerator = -1;
        unsigned int bytes_read = -1;
        if(VCS_GetObject(deviceHandle, NODE_ID, GEAR_RATIO_INDEX, NUMERATOR_SUB_INDEX, 
            &numerator, sizeof(numerator), &bytes_read, &error_code) == 0)
        { failWithCode("Failed to get gear ratio numerator", error_code); }
        return numerator;
    }
    unsigned short EposMotorController::getGearRatioDenominator()
    {
        unsigned int error_code = 0;
        unsigned char DENOMINATOR_SUB_INDEX = 0x02;
        unsigned short denominator = -1;
        unsigned int bytes_read = -1;
        if(VCS_GetObject(deviceHandle, NODE_ID, GEAR_RATIO_INDEX, DENOMINATOR_SUB_INDEX, 
            &denominator, sizeof(denominator), &bytes_read, &error_code) == 0)
        { failWithCode("Failed to get gear ratio denominator", error_code); }
        return denominator;
    }


    void EposMotorController::setMaxVelocity(unsigned int velocity) {
        unsigned int error_code = 0;
        if(VCS_SetMaxProfileVelocity(deviceHandle, NODE_ID, velocity, &error_code) == 0)
        { failWithCode("Failed to set maximum velocity", error_code); }
    }

    double EposMotorController::getMaxVelocity() {
        unsigned int error_code = 0;
        unsigned int velocity = 0;
        if(VCS_GetMaxProfileVelocity(deviceHandle, NODE_ID, &velocity, &error_code) == 0)
        { failWithCode("Failed to set maximum velocity", error_code); }
        return velocity;
    }
    
    double EposMotorController::getMaxAccelDecel() {
        unsigned int error_code = 0;
        unsigned int accel = 0;
        if(VCS_GetMaxAcceleration(deviceHandle, NODE_ID, &accel, &error_code) == 0)
        { failWithCode("Failed to set maximum velocity", error_code); }
        return accel;
    }
        
    void EposMotorController::haltMovement() {
        switch(curOpMode) {
            case OMD_PROFILE_VELOCITY_MODE:
                haltVelocityMovement();
                break;
            case OMD_PROFILE_POSITION_MODE:
                // Fallthrough
            default:
                haltPositionMovement();
                break;
        }
    }

    /************************************
         Profile position movement
    ************************************/
    void EposMotorController::moveToPosition(long position) {
        if(curOpMode == OMD_PROFILE_POSITION_MODE) {
            unsigned int error_code = 0;
            if(VCS_MoveToPosition(deviceHandle,
                NODE_ID, 
                position, 
                1, // absolute = TRUE
                1, // cancel the last one = TRUE
                &error_code) == 0)
            { failWithCode("Failed to command movement to position", error_code, true); }
        } else {
            std::stringstream ss;
            ss << "Cannot move to a position unless in a positioning mode.  Currently in mode " << curOpMode << ".";
            fail(ss.str());
        }
    }

    int EposMotorController::getPosition() {
        unsigned int error_code = 0;
        int position = 0;
        if(VCS_GetPositionIs(deviceHandle, NODE_ID, &position, &error_code) == 0)
        { failWithCode("Failed to get position", error_code); }
        return position;
    }

    long EposMotorController::getTargetPosition() {
        unsigned int error_code = 0;
        long position = 0;
        if(VCS_GetTargetPosition(deviceHandle, NODE_ID, &position, &error_code) == 0)
        { failWithCode("Failed to get position", error_code); }
        return position;
    }

    void EposMotorController::setPositionProfile(unsigned int  velocity, unsigned int  acceleration, unsigned int  deceleration) {
        unsigned int error_code = 0;
        if(VCS_SetPositionProfile(deviceHandle, NODE_ID, velocity, acceleration, deceleration, &error_code) == 0)
        { failWithCode("Failed to set position profile", error_code); }
    }

    void EposMotorController::getPositionProfile(unsigned int *velocity, unsigned int *acceleration, unsigned int *deceleration) {
        unsigned int error_code = 0;
        if(VCS_GetPositionProfile(deviceHandle, NODE_ID, velocity, acceleration, deceleration, &error_code) == 0)
        { failWithCode("Failed to get position profile", error_code); }
    }

    void EposMotorController::haltPositionMovement() {
        unsigned int error_code = 0;
        if(VCS_HaltPositionMovement(deviceHandle, NODE_ID, &error_code) == 0)
        { failWithCode("Failed to halt position movement", error_code, false); }
    }

    /************************************
         Profile velocity movement
    ************************************/
    void EposMotorController::moveWithVelocity(long velocity) {
        if(curOpMode == OMD_PROFILE_VELOCITY_MODE) {
            unsigned int error_code = 0;
            if(VCS_MoveWithVelocity(deviceHandle, NODE_ID, velocity, &error_code) == 0)
            { failWithCode("Failed to command movement to position", error_code, true); }
        } else {
            std::stringstream ss;
            ss << "Cannot move with a velocity unless in profile velocity mode.  Currently in mode " << curOpMode << ".";
            fail(ss.str());
        }
    }
    
    void EposMotorController::setVelocityProfile(unsigned int  acceleration, unsigned int  deceleration) {
        unsigned int error_code = 0;
        if(VCS_SetVelocityProfile(deviceHandle, NODE_ID, acceleration, deceleration, &error_code) == 0)
        { failWithCode("Failed to set velocity profile", error_code); }
    }
    
    void EposMotorController::getVelocityProfile(unsigned int *acceleration, unsigned int *deceleration) {
        unsigned int error_code = 0;
        if(VCS_GetVelocityProfile(deviceHandle, NODE_ID, acceleration, deceleration, &error_code) == 0)
        { failWithCode("Failed to get velocity profile", error_code); }
    }
    
    void EposMotorController::haltVelocityMovement() {
        unsigned int error_code = 0;
        if(VCS_HaltVelocityMovement(deviceHandle, NODE_ID, &error_code) == 0)
        { failWithCode("Failed to halt velocity movement", error_code, false); }
    }

    /************************************
               Error handling
    ************************************/

    void EposMotorController::fail(std::string message, bool disable_motor) {
        if(disable_motor) {
            disable(); // Note that this is likely to throw an exception, so code after this line may not execute.
        }
        throw std::runtime_error(message);
    }

    void EposMotorController::failWithCode(std::string message, int error_code, bool disable_motor) {
        std::stringstream ss;
        ss << message << ": " << lookupError(error_code);
        fail(ss.str(), disable_motor);
    }

    /**
     * Looks up an error from the communications library.
     * This is not to be confused with errors in the motor controller itself.
     * Returns a human-readable string.
     * From section 8.1 (page 8-117) in the EPOS Command Library document.
     */
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
    
    /**
     * Looks up an error from the motor controller.
     * This is not to be confused with errors in the communications library.
     * Returns a human-readable string.
     * From section 4.3 (page 4-19) in the EPOS2 Firmware Specification document.
     */
    std::string EposMotorController::lookupDeviceError(unsigned int error) {
        switch(error) {
            case 0x0000: return "No Error "; break;
            case 0x1000: return "Generic Error"; break;
            case 0x2310: return "Overcurrent Error"; break;
            case 0x3210: return "Overvoltage Error"; break;
            case 0x3220: return "Undervoltage Error"; break;
            case 0x4210: return "Overtemperature Error"; break;
            case 0x5113: return "Logic Supply Voltage Too Low Error"; break;
            case 0x5114: return "Supply Voltage Output Stage Too Low Error"; break;
            case 0x6100: return "Internal Software Error"; break;
            case 0x6320: return "Software Parameter Error"; break;
            case 0x7320: return "Position Sensor Error"; break;
            case 0x8110: return "CAN Overrun Error (Objects lost)"; break;
            case 0x8111: return "CAN Overrun Error"; break;
            case 0x8120: return "CAN Passive Mode Error"; break;
            case 0x8130: return "CAN Life Guarding Error or Heartbeat Error"; break;
            case 0x8150: return "CAN Transmit COB-ID Collision Error"; break;
            case 0x81FD: return "CAN Bus Off Error"; break;
            case 0x81FE: return "CAN Rx Queue Overflow Error"; break;
            case 0x81FF: return "CAN Tx Queue Overflow Error"; break;
            case 0x8210: return "CAN PDO Length Error"; break;
            case 0x8611: return "Following Error"; break;
            case 0xFF01: return "Hall Sensor Error"; break;
            case 0xFF02: return "Index Processing Error"; break;
            case 0xFF03: return "Encoder Resolution Error"; break;
            case 0xFF04: return "Hall Sensor not found Error"; break;
            case 0xFF06: return "Negative Limit Switch Error"; break;
            case 0xFF07: return "Positive Limit Switch Error"; break;
            case 0xFF08: return "Hall Angle Detection Error"; break;
            case 0xFF09: return "Software Position Limit Error"; break;
            case 0xFF0A: return "Position Sensor Breach Error"; break;
            case 0xFF0B: return "System Overloaded Error"; break;
            case 0xFF0C: return "Interpolated Position Mode Error"; break;
            case 0xFF0D: return "Auto Tuning Identification Error"; break;
            case 0xFF0F: return "Gear Scaling Factor Error"; break;
            case 0xFF10: return "Controller Gain Error"; break;
            case 0xFF11: return "Main Sensor Direction Error"; break;
            case 0xFF12: return "Auxiliary Sensor Direction Error"; break;
            default:
            {
                std::stringstream ss;
                ss << "Unknown error: 0x" << std::hex << error;
                return ss.str();
            }
        }
    }
}