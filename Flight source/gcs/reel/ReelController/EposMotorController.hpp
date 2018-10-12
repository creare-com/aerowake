// JDW 2016-2-10
// Copyright Creare 2016
#ifndef EPOS_MOTOR_CONTROLLER_H
#define EPOS_MOTOR_CONTROLLER_H

#include <Definitions.h> // Comes with Epos_Linux_Library
#include <iostream>
#include <string>
#include <stdexcept>
#include <sstream>
namespace gcs {

class EposMotorController {
    private:
        std::string portName;
        void * deviceHandle;
        unsigned int baudRate;
        signed char curOpMode;

        std::string lookupError(unsigned int error);
        std::string lookupDeviceError(unsigned int error);
        void failWithCode(std::string message, int error_code, bool disable_motor = true); // The method called when an error is detected
        void fail(std::string message, bool disable_motor = true); // The method called when an error is detected
        
        void haltPositionMovement(); // Users of this class should call haltMovement() instead
        void haltVelocityMovement(); // Users of this class should call haltMovement() instead
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
        void enable();
        bool isEnabled();
        bool isFaulted();
        unsigned char getDeviceErrorCount();
        std::string getDeviceError(unsigned char device_error_num);
        
        // Configuration (throw an exception on failure)
        void getObject(unsigned short obj_idx, unsigned char obj_sub_idx,void * out_data, unsigned int bytes_to_read);
        void setObject(unsigned short obj_idx, unsigned char obj_sub_idx,void * in_data, unsigned int bytes_to_write);
        void storeAllObjects();
        void setOperatingMode(signed char mode); // Must be one of the "OPM_..." values from Definitions.h
        void setSensorType(unsigned short type); // Must be one of the "ST_..."  values from Definitions.h
        void setEncoderSettings(unsigned int pulses_per_turn=1024, bool invert_polarity=false);
        unsigned short getGearRatioNumerator();
        unsigned short getGearRatioDenominator();
        void setMaxVelocity(unsigned int velocity); // velocity is in RPM after gearbox. Applies to both position and velocity control.
        double getMaxVelocity(); // velocity is in RPM after gearbox. Applies to both position and velocity control.
        double getMaxAccelDecel(); // in RPM/s after gearbox.  This is the limit that the controller will apply to the acceleration/deceleration of the position profile.
        
        // General movement (throw an exception on failure)
        void haltMovement();
        
        // Profile position movement (throw an exception on failure)
        void moveToPosition(long position);
        int getPosition();
        long getTargetPosition();
        void setPositionProfile(unsigned int  velocity, unsigned int  acceleration, unsigned int  deceleration); // velocity/accel/decel is in RPM or RPM/s after gearbox. 
        void getPositionProfile(unsigned int *velocity, unsigned int *acceleration, unsigned int *deceleration); // velocity/accel/decel is in RPM or RPM/s after gearbox. 
        
        // Profile velocity movement (throw an exception on failure)
        void moveWithVelocity(long velocity); // in RPM after gearbox
        void setVelocityProfile(                        unsigned int  acceleration, unsigned int  deceleration); //          accel/decel is in RPM or RPM/s after gearbox. 
        void getVelocityProfile(                        unsigned int *acceleration, unsigned int *deceleration); //          accel/decel is in RPM or RPM/s after gearbox. 
    };
}
#endif // EPOS_MOTOR_CONTROLLER_H
