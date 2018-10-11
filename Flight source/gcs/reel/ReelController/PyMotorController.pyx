""" Adapter classes used to interface with the C++ EposMotorController class """
#import logging
from enum import Enum
from libcpp.string cimport string
from libcpp cimport bool

cdef extern from "include/Definitions.h":
    const signed char OMD_PROFILE_POSITION_MODE     
    const signed char OMD_PROFILE_VELOCITY_MODE     
    const signed char OMD_HOMING_MODE               
    const signed char OMD_INTERPOLATED_POSITION_MODE
    const signed char OMD_POSITION_MODE             
    const signed char OMD_VELOCITY_MODE             
    const signed char OMD_CURRENT_MODE              
    const signed char OMD_MASTER_ENCODER_MODE       
    const signed char OMD_STEP_DIRECTION_MODE       
    
    const unsigned short ST_UNKNOWN                
    const unsigned short ST_INC_ENCODER_3CHANNEL   
    const unsigned short ST_INC_ENCODER_2CHANNEL   
    const unsigned short ST_HALL_SENSORS           
    const unsigned short ST_SSI_ABS_ENCODER_BINARY 
    const unsigned short ST_SSI_ABS_ENCODER_GREY   


cdef extern from "EposMotorController.hpp" namespace "gcs":
    cdef cppclass EposMotorController:
        EposMotorController(string, unsigned int) except +
        # Open/close the specified port. (throw an  exception on failure)
        void open() except +
        void close() except +
        bool isOpen() except +

        # Enable/disable movement. (throw an  exception on failure)
        void getObject(unsigned short obj_idx, unsigned char obj_sub_idx,void * out_data, unsigned int bytes_to_read) except +
        void setObject(unsigned short obj_idx, unsigned char obj_sub_idx,void * in_data, unsigned int bytes_to_write) except +
        void storeAllObjects() except +
        void clearFaultAndEnable() except +
        void clearFault() except +
        void disable() except +
        void enable() except +
        bool isEnabled() except +
        bool isFaulted() except +
        unsigned char getDeviceErrorCount() except +
        string getDeviceError(unsigned char) except +
        
        # Configuration (throw an  exception on failure)
        void setOperatingMode(signed char) except + # Must be one of the "OPM_..." values from Definitions.h
        void setSensorType(unsigned short) except + # Must be one of the "ST_..."  values from Definitions.h
        void setEncoderSettings(unsigned int, bool) except +
        unsigned short getGearRatioNumerator() except +
        unsigned short getGearRatioDenominator() except +
        double getMaxVelocity() except + # velocity is in RPM after gearbox. Applies to both position and velocity control.
        double getMaxAccelDecel() except + # in RPM/s after gearbox.  This is the limit that the controller will apply to the acceleration/deceleration of the position profile.
        
        # General movement (throw an exception on failure)
        void haltMovement() except +
        
        # Profile position movement (throw an  exception on failure)
        void moveToPosition(long) except +
        int getPosition() except +
        long getTargetPosition() except +
        void setMaxVelocity(unsigned int) except + # velocity is in RPM after gearbox. Applies to both position and velocity control.
        void setPositionProfile(unsigned int  , unsigned int  , unsigned int  ) except + # velocity/accel/decel is in RPM or RPM/s after gearbox. 
        void getPositionProfile(unsigned int *, unsigned int *, unsigned int *) except + # velocity/accel/decel is in RPM or RPM/s after gearbox. 
        
        # Profile velocity movement (throw an  exception on failure)
        void moveWithVelocity(long) except + # in RPM after gearbox
        void setVelocityProfile(                unsigned int  , unsigned int  ) except + # accel/decel is in RPM/s after gearbox. 
        void getVelocityProfile(                unsigned int *, unsigned int *) except + # accel/decel is in RPM/s after gearbox. 

# Would use an Enum here, but cython won't let us put non-literals for enum values
SensorType = {
    'UNKNOWN'                : ST_UNKNOWN               ,
    'INC_ENCODER_3CHANNEL'   : ST_INC_ENCODER_3CHANNEL  ,
    'INC_ENCODER_2CHANNEL'   : ST_INC_ENCODER_2CHANNEL  ,
    'HALL_SENSORS'           : ST_HALL_SENSORS          ,
    'SSI_ABS_ENCODER_BINARY' : ST_SSI_ABS_ENCODER_BINARY,
    'SSI_ABS_ENCODER_GREY'   : ST_SSI_ABS_ENCODER_GREY  ,
}

# Would use an Enum here, but cython won't let us put non-literals for enum values
OperatingMode = {
    'PROFILE_POSITION'       : OMD_PROFILE_POSITION_MODE      ,
    'PROFILE_VELOCITY'       : OMD_PROFILE_VELOCITY_MODE      ,
    'HOMING'                 : OMD_HOMING_MODE                ,
    'INTERPOLATED_POSITION'  : OMD_INTERPOLATED_POSITION_MODE ,
    'POSITION'               : OMD_POSITION_MODE              ,
    'VELOCITY'               : OMD_VELOCITY_MODE              ,
    'CURRENT'                : OMD_CURRENT_MODE               ,
    'MASTER_ENCODER'         : OMD_MASTER_ENCODER_MODE        ,
    'STEP_DIRECTION'         : OMD_STEP_DIRECTION_MODE        ,
}
cdef class PyMotorController:
    cdef EposMotorController *_mc
    def __cinit__(self, string usb_port = "USB0", unsigned int baudRate=1000000):
        self._mc = new EposMotorController(usb_port, baudRate)
        self._mc.open()
    def __dealloc__(self):
        del self._mc
        
    # Open/close the specified port. (throw an  exception on failure)
    def open(self):
        return self._mc.open() 
    def close(self):
        return self._mc.close() 
    def isOpen(self):
        return self._mc.isOpen() 

    # Enable/disable movement. (throw an  exception on failure)
    def clearFaultAndEnable(self):
        return self._mc.clearFaultAndEnable() 
    def clearFault(self):
        return self._mc.clearFault() 
    def disable(self):
        return self._mc.disable() 
    def enable(self):
        return self._mc.enable() 
    def isEnabled(self):
        return self._mc.isEnabled() 
    def isFaulted(self):
        return self._mc.isFaulted() 
    def getDeviceErrors(self):
        num_errs = self._mc.getDeviceErrorCount()
        errors = [self._mc.getDeviceError(i) for i in range(0, num_errs)]
        return errors
        
    # Configuration (throw an  exception on failure)
    def getObjectUint16(self, unsigned int idx, unsigned int sub_idx):
        cdef unsigned short ret_val = 0;
        self._mc.getObject(idx, sub_idx, &ret_val, sizeof(ret_val))
        return ret_val
    def setObjectUint16(self, unsigned int idx, unsigned int sub_idx, unsigned short val):
        self._mc.setObject(idx, sub_idx, &val, sizeof(val))
        # Commit to nonvolatile memory immediatelt
        self._mc.storeAllObjects()
    def setOperatingMode(self, string om):
        try:
            return self._mc.setOperatingMode(OperatingMode[om])
        except KeyError:
            raise Exception("Operating Mode must be one of: " + str(OperatingMode.keys()))
    def setSensorType(self, string st):
        try:
            return self._mc.setSensorType(SensorType[st])
        except KeyError:
            raise Exception("Sensor Type must be one of: " + str(SensorType.keys()))
    def setEncoderSettings(self, unsigned int pulses_per_turn=1024, bool invert_polarity=False):
        return self._mc.setEncoderSettings(pulses_per_turn, invert_polarity) 
    def getGearRatioNumerator(self):
        return self._mc.getGearRatioNumerator()
    def getGearRatioDenominator(self):
        return self._mc.getGearRatioDenominator() 
    def getMaxAccelDecel(self):
        """ in RPM/s after gearbox.  This is the limit that the controller will apply to the acceleration/deceleration of the position profile. """
        return self._mc.getMaxAccelDecel()
    def getMaxVelocity(self):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        return self._mc.getMaxVelocity()
        
    # General movement (throw an  exception on failure)
    def haltMovement(self):
        return self._mc.haltMovement() 

    # Movement functions in Profile Position Mode
    def moveToPosition(self, long position):
        return self._mc.moveToPosition(position) 
    def getPosition(self):
        return self._mc.getPosition() 
    def getTargetPosition(self):
        return self._mc.getTargetPosition() 
    def setMaxVelocity(self, unsigned int velocity):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. 
        Note: If this velocity is less than the velocity stored in the motor controller as part of the
        position profile, the velocity in the position profile will be lowered to match it.
        The position profile's velocity will not be raised when this value is raised.
        """
        return self._mc.setMaxVelocity(velocity)
    def setPositionProfile(self, unsigned int  velocity, unsigned int  acceleration, unsigned int  deceleration):
        """ velocity/accel/decel is in RPM or RPM/s after gearbox.  """
        return self._mc.setPositionProfile(velocity, acceleration, deceleration)
    def getPositionProfile(self):
        """ (velocity, accel, decel) is in RPM or RPM/s after gearbox.  """
        cdef unsigned int velocity
        cdef unsigned int acceleration
        cdef unsigned int deceleration
        self._mc.getPositionProfile(&velocity, &acceleration, &deceleration)
        return {'velocity':velocity, 'acceleration':acceleration, 'deceleration':deceleration}

    # Movement functions in Profile Velocity Mode
    def moveWithVelocity(self, long velocity):
        return self._mc.moveWithVelocity(velocity) 
    def setVelocityProfile(self, unsigned int  acceleration, unsigned int  deceleration):
        """ accel/decel is in RPM/s after gearbox """
        return self._mc.setVelocityProfile(acceleration, deceleration)
    def getVelocityProfile(self):
        """ (accel,decel) is in RPM/s after gearbox """
        cdef unsigned int acceleration
        cdef unsigned int deceleration
        self._mc.getVelocityProfile(&acceleration, &deceleration)
        return {'acceleration':acceleration, 'deceleration':deceleration}
