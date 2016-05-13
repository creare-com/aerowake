""" Adapter classes used to interface with the C++ EposMotorController class """
from enum import Enum
from libcpp.string cimport string
from libcpp cimport bool

cdef extern from "EposMotorController.hpp" namespace "gcs":
    cdef cppclass EposMotorController:
        EposMotorController(string, unsigned int) except +
        # Open/close the specified port. (throw an  exception on failure)
        void open() except +
        void close() except +
        bool isOpen() except +

        # Enable/disable movement. (throw an  exception on failure)
        void clearFaultAndEnable() except +
        void clearFault() except +
        void disable() except +
        bool isEnabled() except +
        bool isFaulted() except +
        
        # Configuration (throw an  exception on failure)
        void setOperatingMode(int) except + # This int must be a OperatingMode
        void setSensorType(int) except + # This int must be a SensorType
        void setEncoderSettings(unsigned int, bool) except +
        unsigned short getGearRatioNumerator() except +
        unsigned short getGearRatioDenominator() except +
        
        # Movement (throw an  exception on failure)
        void moveToPosition(long) except +
        int getPosition() except +
        long getTargetPosition() except +
        void setMaxVelocity(unsigned int) except + # velocity is in RPM after gearbox. Applies to both position and velocity control.
        double getMaxVelocity() except + # velocity is in RPM after gearbox. Applies to both position and velocity control.
        void setPositionProfile(unsigned int  , unsigned int  , unsigned int  ) except + # velocity/accel/decel is in RPM or RPM/s after gearbox. 
        void getPositionProfile(unsigned int *, unsigned int *, unsigned int *) except + # velocity/accel/decel is in RPM or RPM/s after gearbox. 
        void haltMovement() except +
        
cdef class PyMotorController:
    class SensorType(Enum): # Copied from Definitions.h
        ST_UNKNOWN                       = 0
        ST_INC_ENCODER_3CHANNEL          = 1
        ST_INC_ENCODER_2CHANNEL          = 2
        ST_HALL_SENSORS                  = 3
        ST_SSI_ABS_ENCODER_BINARY        = 4
        ST_SSI_ABS_ENCODER_GREY          = 5

    class OperatingMode(Enum):
        EPOS_OPMODE_UNKNOWN               = 0
        EPOS_OPMODE_PROFILE_POSITION_MODE = 1

    cdef EposMotorController *_mc
    def __cinit__(self, string usb_port = "USB0", unsigned int baudRate=1000000):
        self._mc = new EposMotorController(usb_port, baudRate)
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
    def isEnabled(self):
        return self._mc.isEnabled() 
    def isFaulted(self):
        return self._mc.isFaulted() 
        
    # Configuration (throw an  exception on failure)
    def setOperatingMode(self, OperatingMode opmode):
        return setOperatingMode(opmode):
    def setSensorType(self, SensorType st)
        return setSensorType(st):
    def setEncoderSettings(self, unsigned int pulses_per_turn=1024, bool invert_polarity=False):
        return setEncoderSettings(pulses_per_turn, invert_polarity) 
    def getGearRatioNumerator(self):
        return getGearRatioNumerator() 
    def getGearRatioDenominator(self):
        return getGearRatioDenominator() 
        
    # Movement (throw an  exception on failure)
    def moveToPosition(self, long position):
        return moveToPosition(position) 
    def getPosition(self):
        return getPosition() 
    def getTargetPosition(self):
        return getTargetPosition() 
    def setMaxVelocity(self, unsigned int velocity):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        return setMaxVelocity(velocity)
    def getMaxVelocity(self):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        return getMaxVelocity()
    def setPositionProfile(self, unsigned int  velocity, unsigned int  acceleration, unsigned int  deceleration):
        """ velocity/accel/decel is in RPM or RPM/s after gearbox.  """
        return setPositionProfile(velocity, acceleration, deceleration)
    def getPositionProfile(self):
        """ (velocity, accel, decel) is in RPM or RPM/s after gearbox.  """
        unsigned int velocity
        unsigned int acceleration
        unsigned int deceleration
        getPositionProfile(&velocity, &acceleration, &deceleration)
        return (velocity, acceleration, deceleration)
    def haltMovement(self):
        return haltMovement() 

    