""" A mock interface to the Cython reelcontroller  """
import logging

class MockPyMotorController:
    def __init__(self):
        self._opened = False
        self._enabled = False
        self._pos = 0;
        self._target_pos = 0;
        self._maxspeed = 0;
        self._profile = (0, 0, 0)
        
    # Open/close the specified port. (throw an  exception on failure)
    def open(self):
        self._opened = True
    def close(self):
        self._opened = False
    def isOpen(self):
        return self._opened

    # Enable/disable movement. (throw an  exception on failure)
    def clearFaultAndEnable(self):
        self._enabled = True
        logging.info("Enabled mock motor controller")
    def clearFault(self):
        pass
    def disable(self):
        logging.info("Disabled mock motor controller")
        self._enabled = False
    def isEnabled(self):
        return self._enabled
    def isFaulted(self):
        return self._mc.isFaulted() 
        
    # Configuration (throw an  exception on failure)
    def setSensorType(self, st):
        pass
    def setEncoderSettings(self, unsigned int pulses_per_turn=1024, bool invert_polarity=False):
        pass
    def getGearRatioNumerator(self):
        return 26 
    def getGearRatioDenominator(self):
        return 1
        
    # Movement (throw an  exception on failure)
    def moveToPosition(self, position):
        self._target_pos = position
    def getPosition(self):
        return self._pos
    def getTargetPosition(self):
        return self._target_pos
    def setActualPosition(self, actual_pos):
        self._pos = actual_pos
    def setMaxVelocity(self, velocity):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        self._maxspeed = velocity
    def getMaxVelocity(self):
        """ velocity is in RPM after gearbox. Applies to both position and velocity control. """
        return self._maxspeed
    def setPositionProfile(self, velocity, acceleration, deceleration):
        """ velocity/accel/decel is in RPM or RPM/s after gearbox.  """
        self._profile = (velocity, acceleration, deceleration)
    def getPositionProfile(self):
        """ (velocity, accel, decel) is in RPM or RPM/s after gearbox.  """
        return self._profile
    def haltMovement(self):
        logging.info("Stopping movement in mock motor controller")

